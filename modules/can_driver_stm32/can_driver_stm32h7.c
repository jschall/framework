#include <common/ctor.h>
#include <hal.h>
#include <string.h>
#include <common/helpers.h>
#include <modules/can/can_driver.h>

#if defined(STM32H7)

#define FDCAN1_IT0_IRQHandler      STM32_FDCAN1_IT0_HANDLER
#define FDCAN1_IT1_IRQHandler      STM32_FDCAN1_IT1_HANDLER
#define FDCAN2_IT0_IRQHandler      STM32_FDCAN2_IT0_HANDLER
#define FDCAN2_IT1_IRQHandler      STM32_FDCAN2_IT1_HANDLER

#define NUM_TX_MAILBOXES 3

#define FDCAN_FRAME_BUFFER_SIZE 4         // Buffer size for 8 bytes data field, in 32-bit words

//Message RAM Allocations in Word lengths
#define MAX_FILTER_LIST_SIZE 80U            //80 element Standard Filter List elements or 40 element Extended Filter List
#define FDCAN_NUM_RXFIFO0_SIZE 104U         //26 Frames
#define FDCAN_TX_FIFO_BUFFER_SIZE 128U      //32 Frames

#define MESSAGE_RAM_END_ADDR 0x4000B5FC


static void can_driver_stm32_start(void* ctx, bool silent, bool auto_retransmit, uint32_t baudrate);
static void can_driver_stm32_stop(void* ctx);
void can_driver_stm32_abort_tx_I(void* ctx, uint8_t mb_idx);
bool can_driver_stm32_load_tx_I(void* ctx, uint8_t mb_idx, struct can_frame_s* frame);

static const struct can_driver_iface_s can_driver_stm32_iface = {
    can_driver_stm32_start,
    can_driver_stm32_stop,
    can_driver_stm32_abort_tx_I,
    can_driver_stm32_load_tx_I,
};

struct message_ram_s {
    // These contain the start addresses of various buffers in CAN SRAM
    uint32_t StandardFilterSA;
    uint32_t ExtendedFilterSA;
    uint32_t RxFIFO0SA;
    uint32_t RxFIFO1SA;
    uint32_t TxBuffer;
    uint32_t EndAddress;
};

struct can_timing_s {
    uint16_t prescaler;
    uint8_t sjw;
    uint8_t bs1;
    uint8_t bs2;
};

struct can_driver_stm32_instance_s {
    struct can_instance_s* frontend;
    struct message_ram_s message_ram;
    uint32_t fdcan_ram_offset; // Offset of next allocatable block in CAN SRAM, in 32-bit words
    uint32_t tx_cb_called_mask;
    FDCAN_GlobalTypeDef* can;
};

static void stm32_can_tx_handler_I(struct can_driver_stm32_instance_s* instance);

static struct can_driver_stm32_instance_s can1_instance;

RUN_ON(CAN_INIT) {
    // TODO make this index configurable and enable multiple instances
    can1_instance.can = FDCAN1;
    can1_instance.frontend = can_driver_register(0, &can1_instance, &can_driver_stm32_iface, FDCAN_TX_FIFO_BUFFER_SIZE/FDCAN_FRAME_BUFFER_SIZE, FDCAN_NUM_RXFIFO0_SIZE/FDCAN_FRAME_BUFFER_SIZE, 4);
}

static bool setupMessageRam(struct can_driver_stm32_instance_s* can_instance)
{
    if (can_instance == NULL) {
        return false;
    }
    uint32_t num_elements = 0;

    can_instance->fdcan_ram_offset = 0;
    // Rx FIFO 0 start address and element count
    num_elements = MIN((FDCAN_NUM_RXFIFO0_SIZE/FDCAN_FRAME_BUFFER_SIZE), 64U);
    if (num_elements) {
        can_instance->can->RXF0C = (num_elements << 16) | (can_instance->fdcan_ram_offset << 2);
        can_instance->message_ram.RxFIFO0SA = SRAMCAN_BASE;
        can_instance->fdcan_ram_offset += num_elements*FDCAN_FRAME_BUFFER_SIZE;
    }

    // Tx FIFO/queue start address and element count
    can_instance->can->TXBC = (NUM_TX_MAILBOXES << 16) | (can_instance->fdcan_ram_offset << 2);
    can_instance->message_ram.TxBuffer = SRAMCAN_BASE + (can_instance->fdcan_ram_offset * 4U);
    can_instance->fdcan_ram_offset += NUM_TX_MAILBOXES*FDCAN_FRAME_BUFFER_SIZE;

    can_instance->message_ram.EndAddress = SRAMCAN_BASE + (can_instance->fdcan_ram_offset * 4U);
    if (can_instance->message_ram.EndAddress > MESSAGE_RAM_END_ADDR) {
        return false;
    }
    return true;
}

static bool computeTimings(const uint32_t target_bitrate,  struct can_timing_s* out_timings)
{
    if (out_timings == NULL) {
        return false;
    }
    memset(out_timings, 0, sizeof(struct can_timing_s));
    if (target_bitrate < 1) {
        return false;
    }

    /*
     * Hardware configuration
     */
    const uint32_t pclk = STM32_PLL1_Q_CK;

    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    if (max_quanta_per_bit >= (MaxBS1 + MaxBS2)) {
        return false;
    }

    static const int MaxSamplePointLocation = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    const uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false;              // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find the values so that the sample point is as close as possible to the optimal value.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    // First attempt with rounding to nearest
    uint8_t bs1 = (uint8_t)(((7 * bs1_bs2_sum - 1) + 4) / 8);
    uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
    uint16_t sample_point_permill = (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));
    
    if (sample_point_permill > MaxSamplePointLocation) {
        // Second attempt with rounding to zero
        bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) / 8);
        bs2 = (uint8_t)(bs1_bs2_sum - bs1);
        sample_point_permill = (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));
    }

    if (!((bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2))) {
        return false;
    }
    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     *
     */
    if ((target_bitrate != (pclk / (prescaler * (1 + bs1 + bs2))))) {
        return false;
    }

    out_timings->prescaler = (uint16_t)(prescaler - 1U);
    out_timings->sjw = 0;                                        // Which means one
    out_timings->bs1 = (uint8_t)(bs1 - 1);
    out_timings->bs2 = (uint8_t)(bs2 - 1);
    return true;
}


static void can_driver_stm32_start(void* ctx, bool silent, bool auto_retransmit, uint32_t baudrate)
{
    struct can_driver_stm32_instance_s* instance = ctx;

    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
    RCC->APB1HENR  |= RCC_APB1HENR_FDCANEN;
    /*
     * IRQ
     */
    nvicEnableVector(FDCAN1_IT0_IRQn, STM32_CAN_CAN1_IRQ_PRIORITY);
    nvicEnableVector(FDCAN1_IT1_IRQn, STM32_CAN_CAN1_IRQ_PRIORITY);

    //CAN Periph Initialisation 
    instance->can->CCCR &= ~FDCAN_CCCR_CSR; // Exit sleep mode
    while ((instance->can->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA) {
        __asm__("nop");
    } //Wait for wake up ack
    instance->can->CCCR |= FDCAN_CCCR_INIT; // Request init
    while ((instance->can->CCCR & FDCAN_CCCR_INIT) == 0) {
        __asm__("nop");
    }
    instance->can->CCCR |= FDCAN_CCCR_CCE; //Enable Config change
    if (auto_retransmit) {
        instance->can->CCCR &= ~FDCAN_CCCR_DAR;
    } else {
        instance->can->CCCR |= FDCAN_CCCR_DAR;
    }
    if (silent) {
        instance->can->CCCR |= FDCAN_CCCR_MON;
    } else {
        instance->can->CCCR &= ~FDCAN_CCCR_MON;
    }
    instance->can->IE = 0;                  // Disable interrupts while initialization is in progress

    struct can_timing_s can_timing;
    if (!computeTimings(baudrate, &can_timing)) {
        instance->can->CCCR &= ~FDCAN_CCCR_INIT;
        return;
    }

    //setup timing register
    //TODO: Do timing calculations for FDCAN
    instance->can->NBTP = ((can_timing.sjw << FDCAN_NBTP_NSJW_Pos)   |
                  (can_timing.bs1 << FDCAN_NBTP_NTSEG1_Pos) |
                  (can_timing.bs2 << FDCAN_NBTP_NTSEG2_Pos) |
                  (can_timing.prescaler << FDCAN_NBTP_NBRP_Pos));

    //RX Config
    instance->can->RXESC = 0; //Set for 8Byte Frames

    //Setup Message RAM
    setupMessageRam(instance);

    //Clear all Interrupts
    instance->can->IR = 0x3FFFFFFF;
    //Enable Interrupts
    instance->can->IE =  FDCAN_IE_TCE |  // Transmit Complete interrupt enable
                FDCAN_IE_RF0NE |  // RX FIFO 0 new message
                FDCAN_IE_RF0FE |  // Rx FIFO 0 FIFO Full
                FDCAN_IE_RF1NE |  // RX FIFO 1 new message
                FDCAN_IE_RF1FE |  // Rx FIFO 1 FIFO Full
                FDCAN_IE_BOE;     // bus off
    instance->can->IE |=  FDCAN_IE_TCFE | FDCAN_IE_PEAE; // Transmit Canceled interrupt enable, it doubles as
                                         // transmit failed in Disabled AutoRetransmission mode.
    instance->can->ILS = FDCAN_ILS_TCL | FDCAN_ILS_TCFL | FDCAN_ILS_PEAE | FDCAN_ILS_BOE;  //Set Line 1 for Transmit Complete Event Interrupt
    instance->can->TXBTIE = (1 << NUM_TX_MAILBOXES) - 1;
    instance->can->ILE = 0x3; // Enable both interrupt handlers

    instance->tx_cb_called_mask = 0;

    //Leave Init
    instance->can->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
    return;
}

static void can_driver_stm32_stop(void* ctx) {
    struct can_driver_stm32_instance_s* instance = ctx;
    (void)instance;

    nvicDisableVector(FDCAN1_IT0_IRQn);
    nvicDisableVector(FDCAN1_IT1_IRQn);

    RCC->APB1HENR  &= ~RCC_APB1HENR_FDCANEN;
}

void can_driver_stm32_abort_tx_I(void* ctx, uint8_t mb_idx) {
    struct can_driver_stm32_instance_s* instance = ctx;

    chDbgCheckClassI();
    chDbgCheck(mb_idx < NUM_TX_MAILBOXES);

    stm32_can_tx_handler_I(instance);

    if (((1 << mb_idx) & instance->tx_cb_called_mask)) {
        // We've already called the completion callback, therefore we fail to abort this mailbox
        return;
    }

    if ((1 << mb_idx) & instance->can->TXBRP) {
        // Transmission is pending or in progress, attempt abort
        instance->can->TXBCR = 1 << mb_idx;
    }
}

#define FDCAN_IDE        (0x40000000U) // Identifier Extension
#define FDCAN_STID_MASK  (0x1FFC0000U) // Standard Identifier Mask
#define FDCAN_EXID_MASK  (0x1FFFFFFFU) // Extended Identifier Mask
#define FDCAN_RTR        (0x20000000U) // Remote Transmission Request
#define FDCAN_DLC_MASK   (0x000F0000U) // Data Length Code

bool can_driver_stm32_load_tx_I(void* ctx, uint8_t mb_idx, struct can_frame_s* frame) {
    struct can_driver_stm32_instance_s* instance = ctx;

    chDbgCheckClassI();

    // if silent just return fail
    if (instance->can->CCCR & FDCAN_CCCR_MON) {
        return false;
    }

    // Copy Frame to RAM
    // Calculate Tx element address
    uint32_t* buffer = (uint32_t *)(instance->message_ram.TxBuffer + (mb_idx * FDCAN_FRAME_BUFFER_SIZE * 4U));

    //Setup Frame ID
    if (frame->IDE) {
        buffer[0] = (FDCAN_IDE | frame->EID);
    } else {
        buffer[0] = (frame->SID << 18);
    }
    if (frame->RTR) {
        buffer[0] |= FDCAN_RTR;
    }
    //Write Data Length Code, and Message Marker
    buffer[1] =  frame->DLC << 16 | mb_idx << 24;

    // Write Frame to the message RAM
    buffer[2] = frame->data32[0];
    buffer[3] = frame->data32[1];

    //Set Add Request
    instance->can->TXBAR = (1 << mb_idx);
    instance->tx_cb_called_mask &= ~(1UL << mb_idx);

    return true;
}

static bool can_driver_stm32_retreive_rx_frame_I(struct can_driver_stm32_instance_s* instance,
                                                struct can_frame_s* frame) {
    uint32_t *frame_ptr;
    uint32_t index;


    //Check if RAM allocated to RX FIFO
    if ((instance->can->RXF0C & FDCAN_RXF0C_F0S) == 0) {
        return false;
    }

    if ((instance->can->RXF0S & FDCAN_RXF0S_F0FL) == 0) {
        return false; //No More messages in FIFO
    } else {
        index = ((instance->can->RXF0S & FDCAN_RXF0S_F0GI) >> 8);
        frame_ptr = (uint32_t *)(instance->message_ram.RxFIFO0SA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));
    }


    // Read the frame contents
    uint32_t id = frame_ptr[0];
    if ((id & FDCAN_IDE) == 0) {
        //Standard ID
        frame->IDE = 0;
        frame->SID = ((id & FDCAN_STID_MASK) >> 18);
    } else {
        //Extended ID
        frame->IDE = 1;
        frame->EID = (id & FDCAN_EXID_MASK);
    }

    if ((id & FDCAN_RTR) != 0) {
        frame->RTR = 1;
    } else {
        frame->RTR = 0;
    }
    frame->DLC = (frame_ptr[1] & FDCAN_DLC_MASK) >> 16;
    uint8_t *data = (uint8_t*)&frame_ptr[2];
    //We only handle Data Length of 8 Bytes for now
    for (uint8_t i = 0; i < 8; i++) {
        frame->data[i] = data[i];
    }

    //Acknowledge the FIFO entry we just read
    instance->can->RXF0A = index;
    return true;
}

static void stm32_can_rx_handler(struct can_driver_stm32_instance_s* instance) {
    systime_t rx_systime = chVTGetSystemTimeX();
    while (true) {
        chSysLockFromISR();
        struct can_frame_s frame;
        if (!can_driver_stm32_retreive_rx_frame_I(instance, &frame)) {
            break;
        }
        can_driver_rx_frame_received_I(instance->frontend, 0, rx_systime, &frame);
        chSysUnlockFromISR();
    }
    chSysUnlockFromISR();
}

static void stm32_can_tx_handler_I(struct can_driver_stm32_instance_s* instance) {
    systime_t t_now = chVTGetSystemTimeX();

    for (uint8_t i = 0; i < NUM_TX_MAILBOXES; i++) {
        if (instance->tx_cb_called_mask & (1UL << i)) {
            // Already called callback for this mailbox
            continue;
        }

        if (instance->can->TXBRP & (1UL << i)) {
            // Transmission is still in progress
            continue;
        }

        if (instance->can->TXBTO & (1UL << i)) {
            // Transmit successful
            instance->tx_cb_called_mask |= (1UL << i);
            can_driver_tx_request_complete_I(instance->frontend, i, true, t_now);
        } else if ((instance->can->CCCR & FDCAN_CCCR_DAR) && (instance->can->TXBCF & (1UL << i)) && can_driver_get_mailbox_transmit_pending(instance->frontend, i) && ((instance->can->ECR & 0xff) == 0)) {
            // Auto-retransmit disabled and transmit cancellation finished and frontend says transmit still desired and no transmit errors
            // Ideally we would use an error flag pertaining to the current frame, rather than the error counter.
            instance->can->TXBAR |= (1UL << i);
            instance->tx_cb_called_mask &= ~(1UL << i);
        } else if (instance->can->TXBCF & (1UL << i)) {
            instance->tx_cb_called_mask |= (1UL << i);
            can_driver_tx_request_complete_I(instance->frontend, i, false, t_now);
        }
    }
}

static void stm32_can_tx_handler(struct can_driver_stm32_instance_s* instance) {
    chSysLockFromISR();
    stm32_can_tx_handler_I(instance);
    chSysUnlockFromISR();
}

static void stm32_can_interrupt_handler(struct can_driver_stm32_instance_s *instance, uint8_t line_index)
{
    if (line_index == 0) {
        if ((instance->can->IR & FDCAN_IR_RF0N) ||
           (instance->can->IR & FDCAN_IR_RF0F)) {
            instance->can->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
            stm32_can_rx_handler(instance);
        }
    } else {
        if (instance->can->IR & FDCAN_IR_TC) {
            instance->can->IR = FDCAN_IR_TC;
            stm32_can_tx_handler(instance);

        }

        if (instance->can->IR & FDCAN_IR_TCF) {
            instance->can->IR = FDCAN_IR_TCF;
            stm32_can_tx_handler(instance);
        }

        if (instance->can->IR & FDCAN_IR_PEA) {
            instance->can->IR = FDCAN_IR_PEA;
            stm32_can_tx_handler(instance);
        }

        if (instance->can->IR & FDCAN_IR_BO) {
            instance->can->IR = FDCAN_IR_BO;
            instance->can->CCCR &= ~FDCAN_CCCR_INIT;
            stm32_can_tx_handler(instance);
        }
    }
}


OSAL_IRQ_HANDLER(STM32_FDCAN1_IT0_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    stm32_can_interrupt_handler(&can1_instance, 0);

    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_FDCAN1_IT1_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    stm32_can_interrupt_handler(&can1_instance, 1);

    OSAL_IRQ_EPILOGUE();
}
#endif //#if defined(STM32H7)
