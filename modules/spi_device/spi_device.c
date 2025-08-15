#include "spi_device.h"

#define FLAG_BIT_VAL(flags, bit) ((flags&bit) != 0 ? 1 : 0)

static SPIDriver* spi_get_driver(uint8_t bus_idx);
static SPIConfig spi_make_config(struct spi_device_s* dev);
static void spi_device_assert_chip_select(struct spi_device_s* dev);
static void spi_device_deassert_chip_select(struct spi_device_s* dev);

bool spi_device_init(struct spi_device_s* dev, uint8_t bus_idx, uint32_t sel_line, uint32_t max_speed_hz, uint8_t data_size, uint8_t flags) {
    if (!dev) {
        return false;
    }

    if (!spi_get_driver(bus_idx)) {
        return false;
    }

    dev->bus_idx = bus_idx;
    dev->sel_line = sel_line;
    dev->max_speed_hz = max_speed_hz;
    dev->data_size = data_size;
    dev->flags = flags;
    dev->bus_acquired = false;

    spi_device_deassert_chip_select(dev);

    if (sel_line == 0) {
        spi_device_begin(dev);
    }

    return true;
}

void spi_device_set_max_speed_hz(struct spi_device_s* dev, uint32_t max_speed_hz) {
    dev->max_speed_hz = max_speed_hz;
}

void spi_device_begin(struct spi_device_s* dev) {
    SPIDriver* spidriver = spi_get_driver(dev->bus_idx);

    if (!spidriver) {
        return;
    }

    dev->spiconf = spi_make_config(dev);

    spiAcquireBus(spidriver);
    dev->bus_acquired = true;

    spiStart(spidriver, &dev->spiconf);
    spi_device_assert_chip_select(dev);
}

void spi_device_end(struct spi_device_s* dev) {
    SPIDriver* spidriver = spi_get_driver(dev->bus_idx);

    if (!spidriver) {
        return;
    }

    spi_device_deassert_chip_select(dev);
    spiReleaseBus(spidriver);
    dev->bus_acquired = false;
}

void spi_device_send(struct spi_device_s* dev, uint32_t n, const void* txbuf) {
    if (n == 0) {
        return; // NOTE: due to a bug in chibios, calling spiSend with a length of 0 blocks forever
    }

    SPIDriver* spidriver = spi_get_driver(dev->bus_idx);

    if (!spidriver) {
        return;
    }

    if (dev->bus_acquired) {
        spiSend(spidriver, n, txbuf);
    } else {
        spi_device_begin(dev);
        spiSend(spidriver, n, txbuf);
        spi_device_end(dev);
    }
}

void spi_device_receive(struct spi_device_s* dev, uint32_t n, void* txbuf) {
    if (n == 0) {
        return; // NOTE: due to a bug in chibios, calling spiSend with a length of 0 blocks forever
    }

    SPIDriver* spidriver = spi_get_driver(dev->bus_idx);

    if (!spidriver) {
        return;
    }

    if (dev->bus_acquired) {
        spiReceive(spidriver, n, txbuf);
    } else {
        spi_device_begin(dev);
        spiReceive(spidriver, n, txbuf);
        spi_device_end(dev);
    }
}

void spi_device_exchange(struct spi_device_s* dev, uint32_t n, const void* txbuf, void* rxbuf) {
    if (n == 0) {
        return; // NOTE: due to a bug in chibios, calling spiSend with a length of 0 blocks forever
    }

    SPIDriver* spidriver = spi_get_driver(dev->bus_idx);

    if (!spidriver) {
        return;
    }

    if (dev->bus_acquired) {
        spiExchange(spidriver, n, txbuf, rxbuf);
    } else {
        spi_device_begin(dev);
        spiExchange(spidriver, n, txbuf, rxbuf);
        spi_device_end(dev);
    }
}

#ifdef STM32H7
#define STM32_SPIx_CLK(x) ((x == 1) ? STM32_SPI1CLK : (x == 2) ? STM32_SPI2CLK : (x == 3) ? STM32_SPI3CLK : (x == 4) ? STM32_SPI4CLK : (x == 5) ? STM32_SPI5CLK : (x == 6) ? STM32_SPI6CLK : 0)
#else
#define STM32_SPIx_CLK(x) ((x == 2 || x == 3) ? (STM32_PCLK1) : (STM32_PCLK2))
#endif
static SPIConfig spi_make_config(struct spi_device_s* dev) {
    uint8_t br_regval;
    for (br_regval=0; br_regval<7; br_regval++) {
#ifdef STM32H7
        if ((uint32_t)STM32_SPIx_CLK(dev->bus_idx)/(2<<(br_regval+1)) < dev->max_speed_hz) {
            break;
        }
#else
        if ((uint32_t)STM32_SPIx_CLK(dev->bus_idx)/(2<<br_regval) < dev->max_speed_hz) {
            break;
        }
#endif
    }

    SPIConfig ret = {0};
#ifdef STM32H7
    ret.cfg1 = ((br_regval&0b111)<<28) | (((dev->data_size-1)&0b1111)<<0);
    ret.cfg2 = (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_CPHA)<<24) | (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_CPOL)<<25) | (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_LSBFIRST)<<23);
#else
    ret.cr1 = ((br_regval&0b111)<<3) | (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_CPHA)<<0) | (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_CPOL)<<1) | (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_LSBFIRST)<<7);
    ret.cr2 = (((dev->data_size-1)&0b1111)<<8);
#endif
    return ret;
}

static void spi_device_assert_chip_select(struct spi_device_s* dev) {
    if (dev->sel_line != 0) {
        if (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_SELPOL)) {
            palSetLine(dev->sel_line);
        } else {
            palClearLine(dev->sel_line);
        }
    }
}

static void spi_device_deassert_chip_select(struct spi_device_s* dev) {
    if (dev->sel_line != 0) {
        if (FLAG_BIT_VAL(dev->flags,SPI_DEVICE_FLAG_SELPOL)) {
            palClearLine(dev->sel_line);
        } else {
            palSetLine(dev->sel_line);
        }
    }
}

static SPIDriver* spi_get_driver(uint8_t bus_idx) {
    switch(bus_idx) {
        case 0:
            return NULL;
#if STM32_SPI_USE_SPI1
        case 1:
            return &SPID1;
#endif
#if STM32_SPI_USE_SPI2
        case 2:
            return &SPID2;
#endif
#if STM32_SPI_USE_SPI3
        case 3:
            return &SPID3;
#endif
#if STM32_SPI_USE_SPI4
        case 4:
            return &SPID4;
#endif
#if STM32_SPI_USE_SPI5
        case 5:
            return &SPID5;
#endif
#if STM32_SPI_USE_SPI6
        case 6:
            return &SPID6;
#endif
        default:
            return NULL;
    }
}
