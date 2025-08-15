#include <common/ctor.h>
#include <hal.h>
#include <modules/can/can_driver.h>

#if defined(STM32F4) || defined(STM32F7)

#if !defined(CAN1) && defined(CAN)
#define CAN1 CAN
#endif

#undef CAN_BTR_BRP
#define CAN_BTR_BRP(n) (n)
#undef CAN_BTR_TS1
#define CAN_BTR_TS1(n) ((n) << 16)
#undef CAN_BTR_TS2
#define CAN_BTR_TS2(n) ((n) << 20)
#undef CAN_BTR_SJW
#define CAN_BTR_SJW(n) ((n) << 24)

#define NUM_TX_MAILBOXES 3
#define NUM_RX_MAILBOXES 2
#define RX_FIFO_DEPTH 3

static void can_driver_stm32_start(void* ctx, bool silent, bool auto_retransmit, uint32_t baudrate);
static void can_driver_stm32_stop(void* ctx);
void can_driver_stm32_abort_tx_mailbox_I(void* ctx, uint8_t mb_idx);
bool can_driver_stm32_load_tx_mailbox_I(void* ctx, uint8_t mb_idx, struct can_frame_s* frame);

static const struct can_driver_iface_s can_driver_stm32_iface = {
    can_driver_stm32_start,
    can_driver_stm32_stop,
    can_driver_stm32_abort_tx_mailbox_I,
    can_driver_stm32_load_tx_mailbox_I,
};

struct can_driver_stm32_instance_s {
    struct can_instance_s* frontend;
    CAN_TypeDef* can;
};

static struct can_driver_stm32_instance_s can1_instance;

RUN_ON(CAN_INIT) {
    // TODO make this index configurable and enable multiple instances
    can1_instance.can = CAN1;
    can1_instance.frontend = can_driver_register(0, &can1_instance, &can_driver_stm32_iface, NUM_TX_MAILBOXES, NUM_RX_MAILBOXES, RX_FIFO_DEPTH);
}

static void can_driver_stm32_start(void* ctx, bool silent, bool auto_retransmit, uint32_t baudrate) {
    struct can_driver_stm32_instance_s* instance = ctx;

    rccEnableCAN1(FALSE);

    instance->can->FMR = (instance->can->FMR & 0xFFFF0000) | CAN_FMR_FINIT;
    instance->can->sFilterRegister[0].FR1 = 0;
    instance->can->sFilterRegister[0].FR2 = 0;
    instance->can->FM1R = 0;
    instance->can->FFA1R = 0;
    instance->can->FS1R = 1;
    instance->can->FA1R = 1;

    instance->can->FMR &= ~CAN_FMR_FINIT;

    nvicEnableVector(STM32_CAN1_TX_NUMBER, STM32_CAN_CAN1_IRQ_PRIORITY);
    nvicEnableVector(STM32_CAN1_RX0_NUMBER, STM32_CAN_CAN1_IRQ_PRIORITY);
    nvicEnableVector(STM32_CAN1_SCE_NUMBER, STM32_CAN_CAN1_IRQ_PRIORITY);

    instance->can->MCR = CAN_MCR_INRQ;
    while((instance->can->MSR & CAN_MSR_INAK) == 0) {
        __asm__("nop");
    }

    // Adapted from libcanard's canardSTM32ComputeCANTimings
    uint8_t bs1;
    uint8_t bs2;
    uint32_t prescaler;

    {
        const uint8_t max_quanta_per_bit = (baudrate >= 1000000) ? 10 : 17;
        const uint32_t prescaler_bs = STM32_PCLK1 / baudrate;

        uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1);

        // Search for the highest valid prescalar value
        while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
            if (bs1_bs2_sum <= 2) {
                return;
            }
            bs1_bs2_sum--;
        }

        prescaler = prescaler_bs / (1 + bs1_bs2_sum);
        if (prescaler < 1 || prescaler > 1024) {
            return;
        }

        // The recommended sample point location is 87.5% or 7/8. Compute the values of BS1 and BS2 that satisfy BS1+BS2 == bs1_bs2_sum and minimize ((1+BS1)/(1+BS1/BS2) - 7/8)
        bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8;

        // Check sample point constraints
        const uint16_t max_sample_point_per_mille = 900;
        const uint16_t min_sample_point_per_mille = (baudrate >= 1000000) ? 750 : 850;

        if (1000 * (1 + bs1) / (1 + bs1_bs2_sum) >= max_sample_point_per_mille) {
            bs1--;
        }

        if (1000 * (1 + bs1) / (1 + bs1_bs2_sum) < min_sample_point_per_mille) {
            bs1++;
        }

        if (1000 * (1 + bs1) / (1 + bs1_bs2_sum) >= max_sample_point_per_mille) {
            return;
        }

        bs2 = bs1_bs2_sum-bs1;
    }

    instance->can->BTR = (silent?CAN_BTR_SILM:0) | CAN_BTR_SJW(0) | CAN_BTR_TS1(bs1-1) | CAN_BTR_TS2(bs2-1) | CAN_BTR_BRP(prescaler - 1);

    instance->can->MCR = CAN_MCR_ABOM | CAN_MCR_AWUM | (auto_retransmit?0:CAN_MCR_NART);

    instance->can->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0; // TODO: review reference manual for other interrupt flags needed
}


static void can_driver_stm32_stop(void* ctx) {
    struct can_driver_stm32_instance_s* instance = ctx;

    instance->can->MCR = 0x00010002;
    instance->can->IER = 0x00000000;

    nvicDisableVector(STM32_CAN1_TX_NUMBER);
    nvicDisableVector(STM32_CAN1_RX0_NUMBER);
    nvicDisableVector(STM32_CAN1_SCE_NUMBER);

    rccDisableCAN1();
}


void can_driver_stm32_abort_tx_mailbox_I(void* ctx, uint8_t mb_idx) {
    struct can_driver_stm32_instance_s* instance = ctx;

    chDbgCheckClassI();

    switch(mb_idx) {
        case 0:
            instance->can->TSR = CAN_TSR_ABRQ0;
            break;
        case 1:
            instance->can->TSR = CAN_TSR_ABRQ1;
            break;
        case 2:
            instance->can->TSR = CAN_TSR_ABRQ2;
            break;
        default:
            break;
    }
}

bool can_driver_stm32_load_tx_mailbox_I(void* ctx, uint8_t mb_idx, struct can_frame_s* frame) {
    struct can_driver_stm32_instance_s* instance = ctx;

    chDbgCheckClassI();

    // if silent just return fail
    if ((instance->can->BTR & CAN_BTR_SILM) != 0) {
        return false;
    }

    CAN_TxMailBox_TypeDef* mailbox = &instance->can->sTxMailBox[mb_idx];

    mailbox->TDTR = frame->DLC;
    mailbox->TDLR = frame->data32[0];
    mailbox->TDHR = frame->data32[1];

    if (frame->IDE) {
        mailbox->TIR = ((uint32_t)frame->EID << 3) | (frame->RTR ? CAN_TI0R_RTR : 0) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;
    } else {
        mailbox->TIR = ((uint32_t)frame->SID << 21) | (frame->RTR ? CAN_TI0R_RTR : 0) | CAN_TI0R_TXRQ;
    }

    return true;
}

static void can_driver_stm32_retreive_rx_frame_I(struct can_frame_s* frame, CAN_FIFOMailBox_TypeDef* mailbox) {
    frame->data32[0] = mailbox->RDLR;
    frame->data32[1] = mailbox->RDHR;
    frame->RTR = (mailbox->RIR & CAN_RI0R_RTR) != 0;
    frame->IDE = (mailbox->RIR & CAN_RI0R_IDE) != 0;
    if (frame->IDE) {
        frame->EID = (mailbox->RIR & (CAN_RI0R_STID|CAN_RI0R_EXID)) >> 3;
    } else {
        frame->SID = (mailbox->RIR & CAN_RI0R_STID) >> 21;
    }
    frame->DLC = mailbox->RDTR & CAN_RDT0R_DLC;
}

static void stm32_can_rx_handler(struct can_driver_stm32_instance_s* instance) {
    systime_t rx_systime = chVTGetSystemTimeX();
    while (true) {
        chSysLockFromISR();
        if ((instance->can->RF0R & CAN_RF0R_FMP0) == 0) {
            chSysUnlockFromISR();
            break;
        }
        struct can_frame_s frame;
        can_driver_stm32_retreive_rx_frame_I(&frame, &instance->can->sFIFOMailBox[0]);
        can_driver_rx_frame_received_I(instance->frontend, 0, rx_systime, &frame);
        instance->can->RF0R = CAN_RF0R_RFOM0;
        chSysUnlockFromISR();
    }

    while (true) {
        chSysLockFromISR();
        if ((instance->can->RF1R & CAN_RF1R_FMP1) == 0) {
            chSysUnlockFromISR();
            break;
        }
        struct can_frame_s frame;
        can_driver_stm32_retreive_rx_frame_I(&frame, &instance->can->sFIFOMailBox[1]);
        can_driver_rx_frame_received_I(instance->frontend, 1, rx_systime, &frame);
        instance->can->RF1R = CAN_RF1R_RFOM1;
        chSysUnlockFromISR();
    }
}

#define CAN_TSR_RQCP(i) ((i==0) ? (CAN_TSR_RQCP0) : (i==1) ? (CAN_TSR_RQCP1) : (i==2) ? (CAN_TSR_RQCP2) : 0)
#define CAN_TSR_TXOK(i) ((i==0) ? (CAN_TSR_TXOK0) : (i==1) ? (CAN_TSR_TXOK1) : (i==2) ? (CAN_TSR_TXOK2) : 0)
#define CAN_TSR_ALST(i) ((i==0) ? (CAN_TSR_ALST0) : (i==1) ? (CAN_TSR_ALST1) : (i==2) ? (CAN_TSR_ALST2) : 0)
#define CAN_TSR_TERR(i) ((i==0) ? (CAN_TSR_TERR0) : (i==1) ? (CAN_TSR_TERR1) : (i==2) ? (CAN_TSR_TERR2) : 0)

static void stm32_can_tx_handler(struct can_driver_stm32_instance_s* instance) {
    systime_t t_now = chVTGetSystemTimeX();

    chSysLockFromISR();

    for (uint8_t i=0; i<NUM_TX_MAILBOXES; i++) {
        if ((instance->can->TSR & CAN_TSR_RQCP(i)) != 0) {
            if ((instance->can->TSR & CAN_TSR_TXOK(i)) != 0) {
                 // Successful transmit
                instance->can->TSR = CAN_TSR_RQCP(i);
                can_driver_tx_request_complete_I(instance->frontend, i, true, t_now);
            } else if ((instance->can->MCR & CAN_MCR_NART) != 0 && can_driver_get_mailbox_transmit_pending(instance->frontend, i) && (instance->can->TSR & CAN_TSR_TERR(i)) == 0) {
                // transmit failed and NART enabled and transmit is still desired and arbitration lost and no errors
                // retransmit this mailbox
                instance->can->TSR = CAN_TSR_RQCP(i);
                instance->can->sTxMailBox[i].TIR |= CAN_TI0R_TXRQ;
            } else {
                // transmit failed
                instance->can->TSR = CAN_TSR_RQCP(i);
                can_driver_tx_request_complete_I(instance->frontend, i, false, t_now);
            }
        }
    }

    chSysUnlockFromISR();
}

OSAL_IRQ_HANDLER(STM32_CAN1_TX_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    stm32_can_tx_handler(&can1_instance);

    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_CAN1_RX0_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    stm32_can_rx_handler(&can1_instance);

    OSAL_IRQ_EPILOGUE();
}

#endif //#if defined(STM32F4) || defined(STM32F7)

