#pragma once

#include "driver/gpio.h"

typedef unsigned int i2c_t;

typedef enum {
    I2C_SPEED_LOW = 0,      /**< low speed mode:     ~10 kbit/s */
    I2C_SPEED_NORMAL,       /**< normal mode:       ~100 kbit/s */
    I2C_SPEED_FAST,         /**< fast mode:         ~400 kbit/s */
    I2C_SPEED_FAST_PLUS,    /**< fast plus mode:   ~1000 kbit/s */
    I2C_SPEED_HIGH,         /**< high speed mode:  ~3400 kbit/s */
} i2c_speed_t;

typedef struct {
    i2c_speed_t speed;          /**< I2C bus speed */
    gpio_num_t scl;             /**< GPIO used as SCL pin */
    gpio_num_t sda;             /**< GPIO used as SDA pin */
} i2c_conf_t;

typedef enum {
    I2C_ADDR10  = 0x01,     /**< use 10-bit device addressing */
    I2C_REG16   = 0x02,     /**< use 16-bit register addressing, big-endian */
    I2C_NOSTOP  = 0x04,     /**< do not issue a STOP condition after transfer */
    I2C_NOSTART = 0x08,     /**< skip START sequence, ignores address field */
} i2c_flags_t;

#ifndef I2C0_SCL
    #define I2C0_SCL 18
#endif
#ifndef I2C0_SDA
    #define I2C0_SDA 5
#endif
#ifndef I2C0_SPEED
    #define I2C0_SPEED I2C_SPEED_FAST_PLUS
#endif

static const i2c_conf_t i2c_config[] = {
#if defined(I2C0_SCL) && defined(I2C0_SDA) && defined(I2C0_SPEED)
    {
        .speed = I2C0_SPEED,
        .scl = gpio_num_t(18),
        .sda = gpio_num_t(5),
    },
#endif
};

#undef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof((a)) / sizeof((a)[0]))
#define I2C_NUMOF       ARRAY_SIZE(i2c_config)
#define I2C_NUMOF_MAX   (2)

#define I2C_READ            (0x0001)
#define I2C_DEV(x)          (x)

#define CHECK_PARAM_RET(cond,err) if (!(cond)) return err;

void i2c_print_config(void);
int i2c_read_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                  void *data, size_t len, uint8_t flags);
int i2c_read_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                 void *data, uint8_t flags);
int i2c_read_byte(i2c_t dev, uint16_t addr, void *data, uint8_t flags);
int i2c_write_byte(i2c_t dev, uint16_t addr, uint8_t data, uint8_t flags);
int i2c_write_regs(i2c_t dev, uint16_t addr, uint16_t reg,
                   const void *data, size_t len, uint8_t flags);
int i2c_write_reg(i2c_t dev, uint16_t addr, uint16_t reg,
                  uint8_t data, uint8_t flags);

void i2c_init(i2c_t dev);
int i2c_acquire(i2c_t dev);
void i2c_release(i2c_t dev);
int IRAM_ATTR i2c_read_bytes(i2c_t dev, uint16_t addr, void *data, size_t len, uint8_t flags);
int IRAM_ATTR i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data, size_t len, uint8_t flags);
void i2c_poweron(i2c_t dev);
void i2c_poweroff(i2c_t dev);
