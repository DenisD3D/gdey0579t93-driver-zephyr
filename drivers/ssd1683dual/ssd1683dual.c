/*
 * Copyright (c) 2022 Andreas Sandberg
 * Copyright (c) 2018-2020 PHYTEC Messtechnik GmbH
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ssd1683dual);

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/display/ssd16xx.h>
#include <display/ssd16xx_regs.h>

/**
 * SSD1683 Dual compatible EPD controller driver.
 */

#define EPD_PANEL_NUMOF_ROWS_PER_PAGE	8
#define SSD1683DUAL_PANEL_FIRST_PAGE	0
#define SSD1683DUAL_PANEL_FIRST_GATE	0
#define SSD1683DUAL_PIXELS_PER_BYTE		8
#define SSD1683DUAL_DEFAULT_TR_VALUE	25U
#define SSD1683DUAL_TR_SCALE_FACTOR		256U


enum ssd1683dual_profile_type {
    SSD1683DUAL_PROFILE_FULL = 0,
    SSD1683DUAL_PROFILE_PARTIAL,
    SSD1683DUAL_NUM_PROFILES,
    SSD1683DUAL_PROFILE_INVALID = SSD1683DUAL_NUM_PROFILES,
};

struct ssd1683dual_quirks {
    /* Gates */
    uint16_t max_width;
    /* Sources */
    uint16_t max_height;
    /* Width (bits) of integer type representing an x coordinate */
    uint8_t pp_width_bits;
    /* Width (bits) of integer type representing a y coordinate */
    uint8_t pp_height_bits;

    /*
     * Device specific flags to be included in
     * SSD16XX_CMD_UPDATE_CTRL2 for a full refresh.
     */
    uint8_t ctrl2_full;
    /*
     * Device specific flags to be included in
     * SSD16XX_CMD_UPDATE_CTRL2 for a partial refresh.
     */
    uint8_t ctrl2_partial;
};

struct ssd1683dual_data {
    bool read_supported;
    uint8_t scan_mode;
    bool blanking_on;
    enum ssd1683dual_profile_type profile;
    enum display_orientation orientation;
};

struct ssd1683dual_dt_array {
    uint8_t *data;
    uint8_t len;
};

struct ssd1683dual_profile {
    struct ssd1683dual_dt_array lut;
    struct ssd1683dual_dt_array gdv;
    struct ssd1683dual_dt_array sdv;
    uint8_t vcom;
    uint8_t bwf;
    uint8_t dummy_line;
    uint8_t gate_line_width;

    bool override_vcom;
    bool override_bwf;
    bool override_dummy_line;
    bool override_gate_line_width;
};

struct ssd1683dual_config {
    const struct device *mipi_dev;
    const struct mipi_dbi_config dbi_config;
    struct gpio_dt_spec busy_gpio;

    const struct ssd1683dual_quirks *quirks;

    struct ssd1683dual_dt_array softstart;

    const struct ssd1683dual_profile *profiles[SSD1683DUAL_NUM_PROFILES];

    uint16_t rotation;
    uint16_t height;
    uint16_t width;
    uint8_t tssv;
};

static int ssd1683dual_set_profile(const struct device *dev,
                                   enum ssd1683dual_profile_type type);

static inline void ssd1683dual_busy_wait(const struct device *dev) {
    const struct ssd1683dual_config *config = dev->config;
    int pin = gpio_pin_get_dt(&config->busy_gpio);

    while (pin > 0) {
        __ASSERT(pin >= 0, "Failed to get pin level");
        k_msleep(SSD16XX_BUSY_DELAY);
        pin = gpio_pin_get_dt(&config->busy_gpio);
    }
}

static inline int ssd1683dual_write_cmd(const struct device *dev, uint8_t cmd,
                                        const uint8_t *data, size_t len) {
    const struct ssd1683dual_config *config = dev->config;
    int err;

    ssd1683dual_busy_wait(dev);

    err = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config,
                                 cmd, data, len);
    mipi_dbi_release(config->mipi_dev, &config->dbi_config);
    return err;
}

// /*static inline int ssd1683dual_write_cmd(const struct device *dev, uint8_t cmd,
//                                         const uint8_t *data, size_t len) {
//     const struct ssd1683dual_config *config = dev->config;
//     int err;
//
//     ssd1683dual_busy_wait(dev);
//
//     // Log the command being sent
//     LOG_INF("CMD: 0x%02X with %zu bytes of data", cmd, len);
//     if (len > 0 && len < 15) {
//         LOG_HEXDUMP_INF(data, len, "Command data:");
//     } else if (len > 0) {
//         LOG_HEXDUMP_INF(data, 15, "Command data (first 15 bytes):");
//     }
//
//     err = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config,
//                                  cmd, data, len);
//     mipi_dbi_release(config->mipi_dev, &config->dbi_config);
//
//     if (err < 0) {
//         LOG_ERR("CMD: 0x%02X failed with error %d", cmd, err);
//     }
//
//     return err;
// }*/

static inline int ssd1683dual_write_uint8(const struct device *dev, uint8_t cmd,
                                          uint8_t data) {
    return ssd1683dual_write_cmd(dev, cmd, &data, 1);
}

static inline int ssd1683dual_read_cmd(const struct device *dev, uint8_t cmd,
                                       uint8_t *data, size_t len) {
    const struct ssd1683dual_config *config = dev->config;
    const struct ssd1683dual_data *dev_data = dev->data;

    if (!dev_data->read_supported) {
        return -ENOTSUP;
    }

    ssd1683dual_busy_wait(dev);

    return mipi_dbi_command_read(config->mipi_dev, &config->dbi_config,
                                 &cmd, 1, data, len);
}

static inline size_t push_x_param(const struct device *dev,
                                  uint8_t *data, uint16_t x) {
    const struct ssd1683dual_config *config = dev->config;

    if (config->quirks->pp_width_bits == 8) {
        data[0] = (uint8_t) x;
        return 1;
    }

    if (config->quirks->pp_width_bits == 16) {
        sys_put_le16(sys_cpu_to_le16(x), data);
        return 2;
    }

    LOG_ERR("Unsupported pp_width_bits %u",
            config->quirks->pp_width_bits);
    return 0;
}

static inline size_t push_y_param(const struct device *dev,
                                  uint8_t *data, uint16_t y) {
    const struct ssd1683dual_config *config = dev->config;

    if (config->quirks->pp_height_bits == 8) {
        data[0] = (uint8_t) y;
        return 1;
    }

    if (config->quirks->pp_height_bits == 16) {
        sys_put_le16(sys_cpu_to_le16(y), data);
        return 2;
    }

    LOG_ERR("Unsupported pp_height_bitsa %u",
            config->quirks->pp_height_bits);
    return 0;
}


static inline int ssd1683dual_set_ram_param(const struct device *dev,
                                            uint16_t sx, uint16_t ex,
                                            uint16_t sy, uint16_t ey) {
    int err;
    uint8_t tmp[4];
    size_t len;

    len = push_x_param(dev, tmp, sx);
    len += push_x_param(dev, tmp + len, ex);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, sy);
    len += push_y_param(dev, tmp + len, ey);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    return 0;
}

static inline int ssd1683dual_set_ram_ptr(const struct device *dev, uint16_t x,
                                          uint16_t y) {
    int err;
    uint8_t tmp[2];
    size_t len;

    len = push_x_param(dev, tmp, x);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CNTR, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, y);
    return ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CNTR, tmp, len);
}

static int ssd1683dual_activate(const struct device *dev, uint8_t ctrl2) {
    int err;

    err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_UPDATE_CTRL2, ctrl2);
    if (err < 0) {
        return err;
    }

    return ssd1683dual_write_cmd(dev, SSD16XX_CMD_MASTER_ACTIVATION, NULL, 0);
}

static int ssd1683dual_update_display(const struct device *dev) {
    const struct ssd1683dual_config *config = dev->config;
    const struct ssd1683dual_data *data = dev->data;
    const struct ssd1683dual_profile *p = config->profiles[data->profile];
    const struct ssd1683dual_quirks *quirks = config->quirks;
    const bool load_lut = !p || p->lut.len == 0;
    const bool load_temp = load_lut && config->tssv;
    const bool partial = data->profile == SSD1683DUAL_PROFILE_PARTIAL;
    const uint8_t update_cmd =
            SSD16XX_CTRL2_ENABLE_CLK |
            SSD16XX_CTRL2_ENABLE_ANALOG |
            (load_lut ? SSD16XX_CTRL2_LOAD_LUT : 0) |
            (load_temp ? SSD16XX_CTRL2_LOAD_TEMPERATURE : 0) |
            (partial ? quirks->ctrl2_partial : quirks->ctrl2_full) |
            SSD16XX_CTRL2_DISABLE_ANALOG |
            SSD16XX_CTRL2_DISABLE_CLK;

    return ssd1683dual_activate(dev, update_cmd);
}

static int ssd1683dual_blanking_off(const struct device *dev) {
    LOG_INF("Blanking off");
    // struct ssd1683dual_data *data = dev->data;
    //
    // if (data->blanking_on) {
    // 	data->blanking_on = false;
    // 	return ssd1683dual_update_display(dev);
    // }

    return 0;
}

static int ssd1683dual_blanking_on(const struct device *dev) {
    LOG_INF("Blanking on");
    // struct ssd1683dual_data *data = dev->data;
    //
    // if (!data->blanking_on) {
    // 	if (ssd1683dual_set_profile(dev, SSD1683DUAL_PROFILE_FULL)) {
    // 		return -EIO;
    // 	}
    // }
    //
    // data->blanking_on = true;

    return 0;
}

static int ssd1683dual_set_window(const struct device *dev,
                                  const uint16_t x, const uint16_t y,
                                  const struct display_buffer_descriptor *desc) {
    const struct ssd1683dual_config *config = dev->config;
    const struct ssd1683dual_data *data = dev->data;
    int err;
    uint16_t x_start;
    uint16_t x_end;
    uint16_t y_start;
    uint16_t y_end;
    uint16_t panel_h = config->height -
                       config->height % EPD_PANEL_NUMOF_ROWS_PER_PAGE;

    if (desc->pitch < desc->width) {
        LOG_ERR("Pitch is smaller than width");
        return -EINVAL;
    }

    if (desc->pitch > desc->width) {
        LOG_ERR("Unsupported mode");
        return -ENOTSUP;
    }

    if (data->orientation == DISPLAY_ORIENTATION_NORMAL ||
        data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
        if ((y + desc->height) > panel_h) {
            LOG_ERR("Buffer out of bounds (height)");
            return -EINVAL;
        }

        if ((x + desc->width) > config->width) {
            LOG_ERR("Buffer out of bounds (width) x=%i desc_width=%i config_width=%i",
                    x, desc->width, config->width);
            return -EINVAL;
        }

        if ((desc->height % EPD_PANEL_NUMOF_ROWS_PER_PAGE) != 0U) {
            LOG_ERR("Buffer height not multiple of %d", EPD_PANEL_NUMOF_ROWS_PER_PAGE);
            return -EINVAL;
        }

        if ((y % EPD_PANEL_NUMOF_ROWS_PER_PAGE) != 0U) {
            LOG_ERR("Y coordinate not multiple of %d", EPD_PANEL_NUMOF_ROWS_PER_PAGE);
            return -EINVAL;
        }
    } else {
        if ((y + desc->height) > config->width) {
            LOG_ERR("Buffer out of bounds (height)");
            return -EINVAL;
        }

        if ((x + desc->width) > panel_h) {
            LOG_ERR("Buffer out of bounds (width) x=%i desc_width=%i config_width=%i",
                    x, desc->width, config->width);
            return -EINVAL;
        }

        if ((desc->width % SSD1683DUAL_PIXELS_PER_BYTE) != 0U) {
            LOG_ERR("Buffer width not multiple of %d", SSD1683DUAL_PIXELS_PER_BYTE);
            return -EINVAL;
        }

        if ((x % SSD1683DUAL_PIXELS_PER_BYTE) != 0U) {
            LOG_ERR("X coordinate not multiple of %d", SSD1683DUAL_PIXELS_PER_BYTE);
            return -EINVAL;
        }
    }

    switch (data->orientation) {
        case DISPLAY_ORIENTATION_NORMAL:
            x_start = (panel_h - 1 - y) / SSD1683DUAL_PIXELS_PER_BYTE;
            x_end = (panel_h - 1 - (y + desc->height - 1)) / SSD1683DUAL_PIXELS_PER_BYTE;
            y_start = x;
            y_end = (x + desc->width - 1);
            break;
        case DISPLAY_ORIENTATION_ROTATED_90:
            x_start = (panel_h - 1 - x) / SSD1683DUAL_PIXELS_PER_BYTE;
            x_end = (panel_h - 1 - (x + desc->width - 1)) / SSD1683DUAL_PIXELS_PER_BYTE;
            y_start = (config->width - 1 - y);
            y_end = (config->width - 1 - (y + desc->height - 1));
            break;
        case DISPLAY_ORIENTATION_ROTATED_180:
            x_start = y / SSD1683DUAL_PIXELS_PER_BYTE;
            x_end = (y + desc->height - 1) / SSD1683DUAL_PIXELS_PER_BYTE;
            y_start = (x + desc->width - 1);
            y_end = x;
            break;
        case DISPLAY_ORIENTATION_ROTATED_270:
            x_start = x / SSD1683DUAL_PIXELS_PER_BYTE;
            x_end = (x + desc->width - 1) / SSD1683DUAL_PIXELS_PER_BYTE;
            y_start = y;
            y_end = (y + desc->height - 1);
            break;
        default:
            return -EINVAL;
    }

    err = ssd1683dual_set_ram_param(dev, x_start, x_end, y_start, y_end);
    if (err < 0) {
        return err;
    }

    err = ssd1683dual_set_ram_ptr(dev, x_start, y_start);
    if (err < 0) {
        return err;
    }

    return 0;
}

static int ssd1683dual_split_buffer(const void *input_buf, size_t input_len,
                                    const struct display_buffer_descriptor *desc,
                                    uint8_t *left_buf, uint8_t *right_buf) {
    const uint16_t Width = (desc->width % 16 == 0) ? (desc->width / 16) : (desc->width / 16 + 1);
    const uint16_t Width1 = (desc->width % 8 == 0) ? (desc->width / 8) : (desc->width / 8 + 1);
    const uint16_t Height = desc->height;

    for (uint16_t y = 0; y < Height; y++) {
        for (uint16_t x = 0; x < Width; x++) {
            left_buf[y * Width + x] = ((const uint8_t *)input_buf)[y * Width1 + x];
        }
    }

    for (uint16_t y = 0; y < Height; y++) {
        for (uint16_t x = 0; x < Width; x++) {
            right_buf[y * Width + x] = ((const uint8_t *)input_buf)[y * Width1 + x + Width - 1];
        }
    }


    // const uint16_t left_width = 396; // pixels 0-395
    // const uint16_t right_width = 396; // pixels 396-791
    // const uint16_t left_bytes_per_row = left_width / SSD1683DUAL_PIXELS_PER_BYTE;
    // const uint16_t right_bytes_per_row = right_width / SSD1683DUAL_PIXELS_PER_BYTE;
    // const uint16_t total_bytes_per_row = total_width / SSD1683DUAL_PIXELS_PER_BYTE;
    //
    // if (total_width != 792 || left_width + right_width != total_width) {
    //     LOG_ERR("Invalid display dimensions for dual controller split");
    //     return -EINVAL;
    // }
    //
    // // Clear output buffers
    // memset(left_buf, 0xFF, left_bytes_per_row * height);
    // memset(right_buf, 0xFF, right_bytes_per_row * height);
    //
    // // Split buffer row by row
    // for (uint16_t y = 0; y < height; y++) {
    //     const uint8_t *src_row = src + (y * total_bytes_per_row);
    //     uint8_t *left_row = left_buf + (y * left_bytes_per_row);
    //     uint8_t *right_row = right_buf + (y * right_bytes_per_row);
    //
    //     // Copy left half (first 396 pixels = 49.5 bytes)
    //     memcpy(left_row, src_row, left_bytes_per_row);
    //
    //     // Copy right half (last 396 pixels = 49.5 bytes)
    //     memcpy(right_row, src_row + left_bytes_per_row, right_bytes_per_row);
    // }

    return 0;
}

static int ssd1683dual_write(const struct device *dev, const uint16_t x,
                             const uint16_t y,
                             const struct display_buffer_descriptor *desc,
                             const void *buf) {
    const struct ssd1683dual_config *config = dev->config;
    const struct ssd1683dual_data *data = dev->data;
    const bool have_partial_refresh =
            config->profiles[SSD1683DUAL_PROFILE_PARTIAL] != NULL;
    const bool partial_refresh = false;
    const uint8_t *buffer = (const uint8_t *) buf;
    const size_t buf_len = MIN(desc->buf_size, desc->height * desc->width / 8);
    int err;

    // Static buffers for dual controllers
    static uint8_t left_controller_buf[13600]; // Max buffer size
    static uint8_t right_controller_buf[13600]; // Max buffer size

    if (buf == NULL || buf_len == 0U) {
        LOG_ERR("Display buffer is not available");
        return -EINVAL;
    }

    LOG_INF("display_buffer_descriptor: buf_size=%u, width=%u, height=%u, pitch=%u",
            desc->buf_size, desc->width, desc->height, desc->pitch);

    // // Print the whole input buffer
    // printk("Input buffer (%zu bytes):\n", buf_len);
    // for (size_t i = 0; i < buf_len; i++) {
    //     printk("%02X ", buffer[i]);
    //     if ((i + 1) % 792 == 0) {
    //         printk("\n");
    //     }
    //     k_msleep(5);  // Small delay to let console catch up
    // }


    // Split the input buffer into left and right controller buffers
    err = ssd1683dual_split_buffer(buf, buf_len, desc,
                                   left_controller_buf, right_controller_buf);
    if (err < 0) {
        return err;
    }

    const size_t controller_buf_len = (desc->width / 2) * desc->height / 8;

    // Print the left input buffer
    // printk("Left Input buffer (%zu bytes):\n", controller_buf_len);
    // for (size_t i = 0; i < controller_buf_len; i++) {
    //     printk("0x%02X, ", left_controller_buf[i]);
    //     if ((i + 1) % 50  == 0) {
    //         printk("\n");
    //     }
    //     k_msleep(1); // Small delay to let console catch up
    // }
    uint8_t tmp[4];
    size_t len;

    len = push_x_param(dev, tmp, 0);
    len += push_x_param(dev, tmp + len, (config->width + 15) / 16 - 1); // = ceil(config->width / 2 / 8 - 1))
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    len += push_y_param(dev, tmp + len, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_x_param(dev, tmp, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CNTR, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CNTR, tmp, len);
    if (err < 0) {
        return err;
    }


    len = push_x_param(dev, tmp, (config->width + 15) / 16 - 1);
    len += push_x_param(dev, tmp + len, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CTRL + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    len += push_y_param(dev, tmp + len, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CTRL + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_x_param(dev, tmp, (config->width + 15) / 16 - 1);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CNTR + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CNTR + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }



    // Write to first controller (left half)
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RAM,
                                left_controller_buf, controller_buf_len);
    if (err < 0) {
        return err;
    }

    memset(left_controller_buf, 0x00, controller_buf_len);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RED_RAM,
                                left_controller_buf, controller_buf_len);
    if (err < 0) {
        return err;
    }

    // Write to second controller (right half)
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RAM + 0x80,
                                right_controller_buf, controller_buf_len);
    if (err < 0) {
        return err;
    }

    memset(right_controller_buf, 0x00, controller_buf_len);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RED_RAM + 0x80,
                                right_controller_buf, controller_buf_len);
    if (err < 0) {
        return err;
    }


    err = ssd1683dual_update_display(dev);
    if (err < 0) {
        return err;
    }

    return 0;
}

int ssd1683dual_read_ram(const struct device *dev, enum ssd16xx_ram ram_type,
                         const uint16_t x, const uint16_t y,
                         const struct display_buffer_descriptor *desc,
                         void *buf) {
    const struct ssd1683dual_data *data = dev->data;
    const size_t buf_len = MIN(desc->buf_size,
                               desc->height * desc->width / 8);
    int err;
    uint8_t ram_ctrl;

    if (!data->read_supported) {
        return -ENOTSUP;
    }

    switch (ram_type) {
        case SSD16XX_RAM_BLACK:
            ram_ctrl = SSD16XX_RAM_READ_CTRL_BLACK;
            break;

        case SSD16XX_RAM_RED:
            ram_ctrl = SSD16XX_RAM_READ_CTRL_RED;
            break;

        default:
            return -EINVAL;
    }

    if (buf == NULL || buf_len == 0U) {
        LOG_ERR("Display buffer is not available");
        return -EINVAL;
    }

    err = ssd1683dual_set_window(dev, x, y, desc);
    if (err < 0) {
        return err;
    }

    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_READ_CTRL,
                                &ram_ctrl, sizeof(ram_ctrl));
    if (err < 0) {
        return err;
    }

    err = ssd1683dual_read_cmd(dev, SSD16XX_CMD_READ_RAM, (uint8_t *) buf,
                               buf_len);
    if (err < 0) {
        return err;
    }

    return 0;
}

static int ssd1683dual_read(const struct device *dev,
                            const uint16_t x, const uint16_t y,
                            const struct display_buffer_descriptor *desc,
                            void *buf) {
    LOG_INF("Read");
    return ssd1683dual_read_ram(dev, SSD16XX_RAM_BLACK, x, y, desc, buf);
}

static void ssd1683dual_get_capabilities(const struct device *dev,
                                         struct display_capabilities *caps) {
    const struct ssd1683dual_config *config = dev->config;
    struct ssd1683dual_data *data = dev->data;

    memset(caps, 0, sizeof(struct display_capabilities));
    caps->x_resolution = config->width;
    caps->y_resolution = config->height -
                         config->height % EPD_PANEL_NUMOF_ROWS_PER_PAGE;
    caps->supported_pixel_formats = PIXEL_FORMAT_MONO10;
    caps->current_pixel_format = PIXEL_FORMAT_MONO10;
    caps->screen_info = SCREEN_INFO_MONO_MSB_FIRST | SCREEN_INFO_EPD;

    // if (data->orientation == DISPLAY_ORIENTATION_NORMAL ||
    // data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
    // caps->screen_info |= SCREEN_INFO_MONO_VTILED;
    // }

    caps->current_orientation = data->orientation;
}

static int ssd1683dual_set_pixel_format(const struct device *dev,
                                        const enum display_pixel_format pf) {
    if (pf == PIXEL_FORMAT_MONO10) {
        return 0;
    }

    LOG_ERR("not supported");
    return -ENOTSUP;
}

static int ssd1683dual_set_orientation(const struct device *dev,
                                       const enum display_orientation orientation) {
    // struct ssd1683dual_data *data = dev->data;
    // int err;
    //
    // if (orientation == DISPLAY_ORIENTATION_NORMAL) {
    // 	data->scan_mode = SSD16XX_DATA_ENTRY_XDYIY;
    // } else if (orientation == DISPLAY_ORIENTATION_ROTATED_90) {
    // 	data->scan_mode = SSD16XX_DATA_ENTRY_XDYDX;
    // } else if (orientation == DISPLAY_ORIENTATION_ROTATED_180) {
    // 	data->scan_mode = SSD16XX_DATA_ENTRY_XIYDY;
    // } else if (orientation == DISPLAY_ORIENTATION_ROTATED_270) {
    // 	data->scan_mode = SSD16XX_DATA_ENTRY_XIYIX;
    // }
    //
    // err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_ENTRY_MODE, data->scan_mode);
    // if (err < 0) {
    // 	return err;
    // }
    //
    // data->orientation = orientation;

    return 0;
}

static int ssd1683dual_clear_cntlr_mem(const struct device *dev, uint8_t ram_cmd) {
    const struct ssd1683dual_config *config = dev->config;
    uint16_t panel_h = config->height / EPD_PANEL_NUMOF_ROWS_PER_PAGE;
    uint16_t last_gate = config->width - 1;
    uint8_t clear_page[64];
    int err;

    /*
     * Clear unusable memory area when the resolution of the panel is not
     * multiple of an octet.
     */
    if (config->height % EPD_PANEL_NUMOF_ROWS_PER_PAGE) {
        panel_h += 1;
    }

    err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_ENTRY_MODE,
                                  SSD16XX_DATA_ENTRY_XIYDY);
    if (err < 0) {
        return err;
    }

    err = ssd1683dual_set_ram_param(dev, SSD1683DUAL_PANEL_FIRST_PAGE,
                                    panel_h - 1, last_gate,
                                    SSD1683DUAL_PANEL_FIRST_GATE);
    if (err < 0) {
        return err;
    }

    err = ssd1683dual_set_ram_ptr(dev, SSD1683DUAL_PANEL_FIRST_PAGE, last_gate);
    if (err < 0) {
        return err;
    }

    if (config->profiles[SSD1683DUAL_PROFILE_PARTIAL] == NULL
        && ram_cmd == SSD16XX_CMD_WRITE_RED_RAM) {
        /* Display doesn't support partial refresh, so it either doesn't matter what is in
         * red Ram, or it needs to be inverted because this is a black/white/red display
         * and for red 0 is off, the opposite of black.
         */
        memset(clear_page, 0x00, sizeof(clear_page));
    } else {
        memset(clear_page, 0xff, sizeof(clear_page));
    }
    for (int h = 0; h < panel_h; h++) {
        size_t x = config->width;

        while (x) {
            size_t l = MIN(x, sizeof(clear_page));

            x -= l;
            err = ssd1683dual_write_cmd(dev, ram_cmd, clear_page, l);
            if (err < 0) {
                return err;
            }
        }
    }

    return 0;
}

static inline int ssd1683dual_load_ws_from_otp_tssv(const struct device *dev) {
    const struct ssd1683dual_config *config = dev->config;

    /*
     * Controller has an integrated temperature sensor or external
     * temperature sensor is connected to the controller.
     */
    LOG_INF("Select and load WS from OTP");
    return ssd1683dual_write_uint8(dev, SSD16XX_CMD_TSENSOR_SELECTION,
                                   config->tssv);
}

static inline int ssd1683dual_load_ws_from_otp(const struct device *dev) {
    int16_t t = (SSD1683DUAL_DEFAULT_TR_VALUE * SSD1683DUAL_TR_SCALE_FACTOR);
    uint8_t tmp[2];
    int err;

    LOG_INF("Load default WS (25 degrees Celsius) from OTP");

    err = ssd1683dual_activate(dev, SSD16XX_CTRL2_ENABLE_CLK);
    if (err < 0) {
        return err;
    }

    /* Load temperature value */
    sys_put_be16(t, tmp);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_TSENS_CTRL, tmp, 2);
    if (err < 0) {
        return err;
    }

    err = ssd1683dual_activate(dev, SSD16XX_CTRL2_DISABLE_CLK);
    if (err < 0) {
        return err;
    }

    return 0;
}


static int ssd1683dual_load_lut(const struct device *dev,
                                const struct ssd1683dual_dt_array *lut) {
    const struct ssd1683dual_config *config = dev->config;

    if (lut && lut->len) {
        LOG_DBG("Using user-provided LUT");
        return ssd1683dual_write_cmd(dev, SSD16XX_CMD_UPDATE_LUT,
                                     lut->data, lut->len);
    } else {
        if (config->tssv) {
            return ssd1683dual_load_ws_from_otp_tssv(dev);
        } else {
            return ssd1683dual_load_ws_from_otp(dev);
        }
    }
}

static int ssd1683dual_set_profile(const struct device *dev,
                                   enum ssd1683dual_profile_type type) {
    const struct ssd1683dual_config *config = dev->config;
    struct ssd1683dual_data *data = dev->data;
    const struct ssd1683dual_profile *p;
    const uint16_t last_gate = config->width - 1;
    uint8_t gdo[3];
    size_t gdo_len;
    int err = 0;

    if (type >= SSD1683DUAL_NUM_PROFILES) {
        return -EINVAL;
    }

    p = config->profiles[type];

    /*
     * The full profile is the only one that always exists. If it
     * hasn't been specified, we use the defaults.
     */
    if (!p && type != SSD1683DUAL_PROFILE_FULL) {
        return -ENOENT;
    }

    if (type == data->profile) {
        return 0;
    }

    /*
     * Perform a soft reset to make sure registers are reset. This
     * will leave the RAM contents intact.
     */
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_SW_RESET, NULL, 0);
    if (err < 0) {
        return err;
    }

    // gdo_len = push_y_param(dev, gdo, last_gate);
    // gdo[gdo_len++] = 0U;
    // err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_GDO_CTRL, gdo, gdo_len);
    // if (err < 0) {
    // 	return err;
    // }
    //
    // if (config->softstart.len) {
    // 	err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_SOFTSTART,
    // 				config->softstart.data,
    // 				config->softstart.len);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // err = ssd1683dual_load_lut(dev, p ? &p->lut : NULL);
    // if (err < 0) {
    // 	return err;
    // }
    //
    // if (p && p->override_dummy_line) {
    // 	err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_DUMMY_LINE,
    // 				  p->dummy_line);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // if (p && p->override_gate_line_width) {
    // 	err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_GATE_LINE_WIDTH,
    // 				  p->override_gate_line_width);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // if (p && p->gdv.len) {
    // 	LOG_DBG("Setting GDV");
    // 	err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_GDV_CTRL,
    // 				p->gdv.data, p->gdv.len);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // if (p && p->sdv.len) {
    // 	LOG_DBG("Setting SDV");
    // 	err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_SDV_CTRL,
    // 				p->sdv.data, p->sdv.len);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // if (p && p->override_vcom) {
    // 	LOG_DBG("Setting VCOM");
    // 	err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_VCOM_VOLTAGE,
    // 				&p->vcom, 1);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // if (p && p->override_bwf) {
    // 	LOG_DBG("Setting BWF");
    // 	err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_BWF_CTRL,
    // 				&p->bwf, 1);
    // 	if (err < 0) {
    // 		return err;
    // 	}
    // }
    //
    // err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_ENTRY_MODE, data->scan_mode);
    // if (err < 0) {
    // 	return err;
    // }

    data->profile = type;

    return 0;
}

static int ssd1683dual_controller_init(const struct device *dev) {
    const struct ssd1683dual_config *config = dev->config;
    struct ssd1683dual_data *data = dev->data;
    enum display_orientation orientation;
    int err;

    LOG_DBG("");

    data->blanking_on = false;
    data->profile = SSD1683DUAL_PROFILE_INVALID;

    err = mipi_dbi_reset(config->mipi_dev, SSD16XX_RESET_DELAY);
    if (err < 0) {
        return err;
    }

    k_msleep(SSD16XX_RESET_DELAY);

    err = ssd1683dual_set_profile(dev, SSD1683DUAL_PROFILE_FULL);
    if (err < 0) {
        return err;
    }

    data->scan_mode = SSD16XX_DATA_ENTRY_XIYDX;
    err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_ENTRY_MODE, data->scan_mode);
    if (err < 0) {
        return err;
    }

    uint8_t tmp[4];
    size_t len;

    len = push_x_param(dev, tmp, 0);
    len += push_x_param(dev, tmp + len, (config->width + 15) / 16 - 1); // = ceil(config->width / 2 / 8 - 1))
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    len += push_y_param(dev, tmp + len, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_x_param(dev, tmp, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CNTR, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CNTR, tmp, len);
    if (err < 0) {
        return err;
    }

    ssd1683dual_busy_wait(dev);


    // Second controller
    err = ssd1683dual_write_uint8(dev, SSD16XX_CMD_ENTRY_MODE + 0x80, SSD16XX_DATA_ENTRY_XDYDX);
    if (err < 0) {
        return err;
    }

    len = push_x_param(dev, tmp, (config->width + 15) / 16 - 1);
    len += push_x_param(dev, tmp + len, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CTRL + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    len += push_y_param(dev, tmp + len, 0);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CTRL + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_x_param(dev, tmp, (config->width + 15) / 16 - 1);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CNTR + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, config->height - 1);
    err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CNTR + 0x80, tmp, len);
    if (err < 0) {
        return err;
    }

    ssd1683dual_busy_wait(dev);


    // Clear screen
    const int total_bytes = 13600;
    const int chunk_size = 256; // Smaller buffer size

    // Clear first controller (0x24 command) - write 0xFF
    uint8_t clear_data[chunk_size];
    memset(clear_data, 0xFF, chunk_size);
    for (int offset = 0; offset < total_bytes; offset += chunk_size) {
        int bytes_to_write = MIN(chunk_size, total_bytes - offset);
        err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RAM, clear_data, bytes_to_write);
        if (err < 0) {
            return err;
        }
    }

    // Clear first controller (0x26 command) - write 0x00
    memset(clear_data, 0x00, chunk_size);
    for (int offset = 0; offset < total_bytes; offset += chunk_size) {
        int bytes_to_write = MIN(chunk_size, total_bytes - offset);
        err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RED_RAM, clear_data, bytes_to_write);
        if (err < 0) {
            return err;
        }
    }

    // Clear second controller (0xA4 command) - write 0xFF
    memset(clear_data, 0xFF, chunk_size);
    for (int offset = 0; offset < total_bytes; offset += chunk_size) {
        int bytes_to_write = MIN(chunk_size, total_bytes - offset);
        err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RAM + 0x80, clear_data, bytes_to_write);
        if (err < 0) {
            return err;
        }
    }

    // Clear second controller (0xA6 command) - write 0x00
    memset(clear_data, 0x00, chunk_size);
    for (int offset = 0; offset < total_bytes; offset += chunk_size) {
        int bytes_to_write = MIN(chunk_size, total_bytes - offset);
        err = ssd1683dual_write_cmd(dev, SSD16XX_CMD_WRITE_RED_RAM + 0x80, clear_data, bytes_to_write);
        if (err < 0) {
            return err;
        }
    }

    // Turn on display
    ssd1683dual_update_display(dev);


    // err = ssd1683dual_clear_cntlr_mem(dev, SSD16XX_CMD_WRITE_RAM);
    // if (err < 0) {
    // 	return err;
    // }
    //
    // err = ssd1683dual_clear_cntlr_mem(dev, SSD16XX_CMD_WRITE_RED_RAM);
    // if (err < 0) {
    // 	return err;
    // }
    //
    // if (config->rotation == 0U) {
    // 	orientation = DISPLAY_ORIENTATION_NORMAL;
    // } else if (config->rotation == 90U) {
    // 	orientation = DISPLAY_ORIENTATION_ROTATED_90;
    // } else if (config->rotation == 180U) {
    // 	orientation = DISPLAY_ORIENTATION_ROTATED_180;
    // } else {
    // 	orientation = DISPLAY_ORIENTATION_ROTATED_270;
    // }
    //
    // err = ssd1683dual_set_orientation(dev, orientation);
    // if (err < 0) {
    // 	return err;
    // }

    // err = ssd1683dual_update_display(dev);
    // if (err < 0) {
    // 	return err;
    // }

    return 0;
}

static int ssd1683dual_init(const struct device *dev) {
    const struct ssd1683dual_config *config = dev->config;
    struct ssd1683dual_data *data = dev->data;
    int err;

    LOG_DBG("");

    if (!device_is_ready(config->mipi_dev)) {
        LOG_ERR("MIPI Device not ready");
        return -ENODEV;
    }

    data->read_supported =
            (config->dbi_config.config.operation & SPI_HALF_DUPLEX) != 0;

    if (!gpio_is_ready_dt(&config->busy_gpio)) {
        LOG_ERR("Busy GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->busy_gpio, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Failed to configure busy GPIO");
        return err;
    }

    if (config->width > config->quirks->max_width ||
        config->height > config->quirks->max_height) {
        LOG_ERR("Display size out of range.");
        return -EINVAL;
    }

    return ssd1683dual_controller_init(dev);
}

static DEVICE_API(display, ssd1683dual_driver_api) = {
    .blanking_on = ssd1683dual_blanking_on,
    .blanking_off = ssd1683dual_blanking_off,
    .write = ssd1683dual_write,
    .read = ssd1683dual_read,
    .get_capabilities = ssd1683dual_get_capabilities,
    .set_pixel_format = ssd1683dual_set_pixel_format,
    .set_orientation = ssd1683dual_set_orientation,
};

#if DT_HAS_COMPAT_STATUS_OKAY(solomon_ssd1683dual)
static struct ssd1683dual_quirks quirks_solomon_ssd1683dual = {
	.max_width = 800,
	.max_height = 300,
	.pp_width_bits = 8,
	.pp_height_bits = 16,
	.ctrl2_full = SSD16XX_GEN2_CTRL2_DISPLAY,
	.ctrl2_partial = SSD16XX_GEN2_CTRL2_DISPLAY | SSD16XX_GEN2_CTRL2_MODE2,
};
#endif

#define SOFTSTART_ASSIGN(n)						\
		.softstart = {						\
			.data = softstart_##n,				\
			.len = sizeof(softstart_##n),			\
		},

#define SSD1683DUAL_MAKE_ARRAY_OPT(n, p)					\
	static uint8_t data_ ## n ## _ ## p[] = DT_PROP_OR(n, p, {})

#define SSD1683DUAL_ASSIGN_ARRAY(n, p)					\
	{								\
		.data = data_ ## n ## _ ## p,				\
		.len = sizeof(data_ ## n ## _ ## p),			\
	}

#define SSD1683DUAL_PROFILE(n)						\
	SSD1683DUAL_MAKE_ARRAY_OPT(n, lut);					\
	SSD1683DUAL_MAKE_ARRAY_OPT(n, gdv);					\
	SSD1683DUAL_MAKE_ARRAY_OPT(n, sdv);					\
									\
	static const struct ssd1683dual_profile ssd1683dual_profile_ ## n = {	\
		.lut = SSD1683DUAL_ASSIGN_ARRAY(n, lut),			\
		.gdv = SSD1683DUAL_ASSIGN_ARRAY(n, gdv),			\
		.sdv = SSD1683DUAL_ASSIGN_ARRAY(n, sdv),			\
		.vcom = DT_PROP_OR(n, vcom, 0),				\
		.override_vcom = DT_NODE_HAS_PROP(n, vcom),		\
		.bwf = DT_PROP_OR(n, border_waveform, 0),		\
		.override_bwf = DT_NODE_HAS_PROP(n, border_waveform),	\
		.dummy_line = DT_PROP_OR(n, dummy_line, 0),		\
		.override_dummy_line = DT_NODE_HAS_PROP(n, dummy_line),	\
		.gate_line_width = DT_PROP_OR(n, gate_line_width, 0),	\
		.override_gate_line_width = DT_NODE_HAS_PROP(		\
			n, gate_line_width),				\
	};


#define _SSD1683DUAL_PROFILE_PTR(n) &ssd1683dual_profile_ ## n

#define SSD1683DUAL_PROFILE_PTR(n)						\
	COND_CODE_1(DT_NODE_EXISTS(n),					\
		    (_SSD1683DUAL_PROFILE_PTR(n)),				\
		    NULL)

#define SSD1683DUAL_DEFINE(n, quirks_ptr)					\
	SSD1683DUAL_MAKE_ARRAY_OPT(n, softstart);				\
									\
	DT_FOREACH_CHILD(n, SSD1683DUAL_PROFILE);				\
									\
	static const struct ssd1683dual_config ssd1683dual_cfg_ ## n = {	\
		.mipi_dev = DEVICE_DT_GET(DT_PARENT(n)),                \
		.dbi_config = {                                         \
			.mode = MIPI_DBI_MODE_SPI_4WIRE,                \
			.config = MIPI_DBI_SPI_CONFIG_DT(n,             \
				SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |  \
				SPI_HOLD_ON_CS | SPI_LOCK_ON, 0),       \
		},                                                      \
		.busy_gpio = GPIO_DT_SPEC_GET(n, busy_gpios),		\
		.quirks = quirks_ptr,					\
		.height = DT_PROP(n, height),				\
		.width = DT_PROP(n, width),				\
		.rotation = DT_PROP(n, rotation),			\
		.tssv = DT_PROP_OR(n, tssv, 0),				\
		.softstart = SSD1683DUAL_ASSIGN_ARRAY(n, softstart),	\
		.profiles = {						\
			[SSD1683DUAL_PROFILE_FULL] =			\
				SSD1683DUAL_PROFILE_PTR(DT_CHILD(n, full)),	\
			[SSD1683DUAL_PROFILE_PARTIAL] =			\
				SSD1683DUAL_PROFILE_PTR(DT_CHILD(n, partial)),\
		},							\
	};								\
									\
	static struct ssd1683dual_data ssd1683dual_data_ ## n;			\
									\
	DEVICE_DT_DEFINE(n,						\
			 ssd1683dual_init, NULL,				\
			 &ssd1683dual_data_ ## n,				\
			 &ssd1683dual_cfg_ ## n,				\
			 POST_KERNEL,					\
			 CONFIG_DISPLAY_INIT_PRIORITY,			\
			 &ssd1683dual_driver_api)

DT_FOREACH_STATUS_OKAY_VARGS(solomon_ssd1683dual, SSD1683DUAL_DEFINE,
                             &quirks_solomon_ssd1683dual);
