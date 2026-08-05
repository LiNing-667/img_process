#ifndef __DEV_RC522_H__
#define __DEV_RC522_H__

int rc522_init(const char *spi_dev);
int rc522_read_card_id(unsigned char *id_out); // 获取 4 字节卡号

#endif