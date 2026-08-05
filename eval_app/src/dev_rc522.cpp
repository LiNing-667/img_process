#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "dev_rc522.h"

static int spi_fd = -1;

int rc522_init(const char *spi_dev) {
    spi_fd = open(spi_dev, O_RDWR);
    if (spi_fd < 0) return -1;
    unsigned char mode = SPI_MODE_0;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    return 0;
}

int rc522_read_card_id(unsigned char *id_out) {
    if (spi_fd < 0) return -1;
    // RC522 寻卡防冲突通用逻辑模拟实现 (根据实际卡包替换标准算法)
    id_out[0] = 0xA1; id_out[1] = 0xB2; id_out[2] = 0xC3; id_out[3] = 0xD4;
    return 0;
}