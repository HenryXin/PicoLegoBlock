/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};


#define SW0_NODE	DT_ALIAS(sw0)
#define I2C0_NODE DT_NODELABEL(colorsensor)
#define I2C1_NODE_LED_CMD DT_NODELABEL(tm1650ledcmd)
#define I2C1_NODE_LED_DATA1 DT_NODELABEL(tm1650leddata1)
#define I2C1_NODE_LED_DATA2 DT_NODELABEL(tm1650leddata2)
#define I2C1_NODE_LED_DATA3 DT_NODELABEL(tm1650leddata3)
#define I2C1_NODE_LED_DATA4 DT_NODELABEL(tm1650leddata4)

static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
static const struct pwm_dt_spec pwm_led3 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led3));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
static const struct i2c_dt_spec dev_i2c_led_cmd = I2C_DT_SPEC_GET(I2C1_NODE_LED_CMD);
static const struct i2c_dt_spec dev_i2c_led_data1 = I2C_DT_SPEC_GET(I2C1_NODE_LED_DATA1);
static const struct i2c_dt_spec dev_i2c_led_data2 = I2C_DT_SPEC_GET(I2C1_NODE_LED_DATA2);
static const struct i2c_dt_spec dev_i2c_led_data3 = I2C_DT_SPEC_GET(I2C1_NODE_LED_DATA3);
static const struct i2c_dt_spec dev_i2c_led_data4 = I2C_DT_SPEC_GET(I2C1_NODE_LED_DATA4);

#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)


//Device Address
//ADDR->Low
#define RGB_2           0x00
#define ADDR            0x53
#define W_ADDR_L        0x66
#define R_ADDR_L        0x67
//ADDR->High
#define RGB_1           0x01
#define W_ADDR_H        0x98
#define R_ADDR_H        0x99

//Register Address
#define ControlReg_Addr 0x00
#define RGBCON_Addr     0x04

#define ID_Addr         0x06

#define CDATAL_Addr     0x12
#define CDATAH_Addr     0x13

#define RDATAL_Addr     0x10
#define RDATAM_Addr     0x11
#define RDATAH_Addr     0x12

#define GDATAL_Addr     0x0D
#define GDATAM_Addr     0x0E
#define GDATAH_Addr     0x0F

#define BDATAL_Addr     0x13
#define BDATAM_Addr     0x14
#define BDATAH_Addr     0x15
/*  Confi Arg  */
//Control Register 
#define RST             0x00

// I2C Address definitions
#define DISPLAY_CMD_ADDR     0x24  // Command address for display control
#define DISPLAY_DATA_ADDR1   0x34  // Data address for digit 1
#define DISPLAY_DATA_ADDR2   0x35  // Data address for digit 2
#define DISPLAY_DATA_ADDR3   0x36  // Data address for digit 3
#define DISPLAY_DATA_ADDR4   0x37  // Data address for digit 4

// Display control commands
#define DISPLAY_ON   0x01
#define DISPLAY_OFF  0x00

// Segment patterns for digits 0-9 and some special characters
const uint8_t digitPatterns[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71  // F
};

// Function prototypes
void TM1650_init(void);
void TM1650_setBrightness(uint8_t brightness);
void TM1650_displayDigit(uint8_t position, uint8_t digit);
void TM1650_displayTest(void);
void TM1650_writeI2C(uint8_t addr, uint8_t data);

static struct gpio_callback button_cb_data;

uint8_t led_selected = 0;
unsigned int Red = 0;
unsigned int Green  = 0;
unsigned int Blue = 0;
unsigned int val_red = 0;
unsigned int val_green = 0;
unsigned int val_blue = 0;

void RGB_Config(void);
void ReadColor();

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	led_selected++;
	if(led_selected>2) {
		led_selected = 0;
	}
}


int main(void)
{
	uint32_t max_period;
	uint32_t period;
	uint8_t dir = 0U;
	int ret;

	uint32_t count = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 0;
		}

		ret = adc_channel_setup_dt(&adc_channels[i]);
		if (ret < 0) {
			printk("Could not setup channel #%d (%d)\n", i, ret);
			return 0;
		}
	}


	printk("PWM-based blinky\n");

	if (!pwm_is_ready_dt(&pwm_led1)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led1.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&pwm_led2)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led2.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&pwm_led3)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led3.dev->name);
		return 0;
	}

	
	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}
	
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return;
	}

	if (!device_is_ready(dev_i2c_led_cmd.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c_led_cmd.bus->name);
		return;
	}

	if (!device_is_ready(dev_i2c_led_data1.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c_led_data1.bus->name);
		return;
	}

	if (!device_is_ready(dev_i2c_led_data2.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c_led_data2.bus->name);
		return;
	}

	if (!device_is_ready(dev_i2c_led_data3.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c_led_data3.bus->name);
		return;
	}

	if (!device_is_ready(dev_i2c_led_data4.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c_led_data4.bus->name);
		return;
	}
	

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	RGB_Config();
	TM1650_init();
	/*
	 * In case the default MAX_PERIOD value cannot be set for
	 * some PWM hardware, decrease its value until it can.
	 *
	 * Keep its value at least MIN_PERIOD * 4 to make sure
	 * the sample changes frequency at least once.
	 */
	printk("Calibrating for channel %d...\n", pwm_led1.channel);
	max_period = MAX_PERIOD;
	while (pwm_set_dt(&pwm_led1, max_period, max_period / 2U)) {
		max_period /= 2U;
		if (max_period < (4U * MIN_PERIOD)) {
			printk("Error: PWM device "
			       "does not support a period at least %lu\n",
			       4U * MIN_PERIOD);
			return 0;
		}
	}

	printk("Done calibrating; maximum/minimum periods %u/%lu nsec\n",
	       max_period, MIN_PERIOD);

	period = max_period;
	while (1) {
		if(led_selected == 0) {
			ret = pwm_set_dt(&pwm_led1, period, period / 2U);
			if (ret) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
			ret = pwm_set_dt(&pwm_led2, period, 0);
			ret = pwm_set_dt(&pwm_led3, period, 0);
			printk("Using period %d\n", period);
		} 
		else if(led_selected == 1) 
		{
			ret = pwm_set_dt(&pwm_led2, period, period / 2U);
			if (ret) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
			ret = pwm_set_dt(&pwm_led1, period, 0);
			ret = pwm_set_dt(&pwm_led3, period, 0);
		}
		else {
			ret = pwm_set_dt(&pwm_led3, period, period / 2U);
			if (ret) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
			ret = pwm_set_dt(&pwm_led1, period, 0);
			ret = pwm_set_dt(&pwm_led2, period, 0);
		}
		period = dir ? (period * 2U) : (period / 2U);
		if (period > max_period) {
			period = max_period / 2U;
			dir = 0U;
		} else if (period < MIN_PERIOD) {
			period = MIN_PERIOD * 2U;
			dir = 1U;
		}

		int val = gpio_pin_get_dt(&button);

		if (val >= 0) {
			//gpio_pin_set_dt(&led, val);
			printk("Button value: %d\n", val);
		}
		ReadColor();
		TM1650_displayTest();

		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int32_t val_mv;

			printk("- %s, channel %d: ",
			       adc_channels[i].dev->name,
			       adc_channels[i].channel_id);

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			ret = adc_read_dt(&adc_channels[i], &sequence);
			if (ret < 0) {
				printk("Could not read (%d)\n", ret);
				continue;
			}

			/*
			 * If using differential mode, the 16 bit value
			 * in the ADC sample buffer should be a signed 2's
			 * complement value.
			 */
			if (adc_channels[i].channel_cfg.differential) {
				val_mv = (int32_t)((int16_t)buf);
			} else {
				val_mv = (int32_t)buf;
			}
			printk("%"PRId32, val_mv);
			ret = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
			/* conversion to mV may not be supported, skip if not */
			if (ret < 0) {
				printk(" (value in mV not available)\n");
			} else {
				printk(" = %"PRId32" mV\n", val_mv);
			}
		}
		k_sleep(K_SECONDS(4U));
	}
	return 0;
}

void RGB_Config(void)
{
	int ret;
	uint8_t config[2] = {ControlReg_Addr,0x06};
    
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,config[0]);
	}

    config[0] = 0x04;
	config[1] = 0x41;
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,config[0]);
	}
    
	config[0] = 0x05;
	config[1] = 0x01;
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at reg. %x \n\r", dev_i2c.addr,config[0]);
	}
}

static uint8_t IIC_ReadReg(uint8_t bReg){
	uint8_t Temp = 0;
	int ret;

	ret = i2c_write_dt(&dev_i2c, &bReg, sizeof(bReg));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at reg.\n\r", dev_i2c.addr);
	}

	ret = i2c_read_dt(&dev_i2c, &Temp, sizeof(Temp));
	if(ret != 0){
		printk("Failed to read from I2C device address %x at Reg.\n\r", dev_i2c.addr);
	}
	return Temp;
}       

static uint32_t IIC_Read2Reg(uint8_t bReg){
	uint32_t Temp;
	uint32_t TempData = 0;
	int ret;

	ret = i2c_write_dt(&dev_i2c, &bReg, sizeof(bReg));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at reg.\n\r", dev_i2c.addr);
	}

	ret = i2c_read_dt(&dev_i2c, &Temp, sizeof(Temp));
	if(ret != 0){
		printk("Failed to read from I2C device address %x at Reg.\n\r", dev_i2c.addr);
	}
	TempData |= Temp;    		
		
					
	ret = i2c_read_dt(&dev_i2c, &Temp, sizeof(Temp));
	if(ret != 0){
		printk("Failed to read from I2C device address %x at Reg.\n\r", dev_i2c.addr);
	} 
	TempData |=(unsigned int)(Temp << 8);      

	return TempData;
} 

void ReadColor()                 //Get color
{
	int ret;
    int index = 0;
    char ColorData[6] = {0};
	uint32_t Temp;

    //Red
	Temp = IIC_Read2Reg(0x10);
	ColorData[index++] = Temp & 0xff;
	ColorData[index++] = (Temp >> 8) & 0xff;
    //Green
    Temp = IIC_Read2Reg(0x0D);
	ColorData[index++] = Temp & 0xff;
	ColorData[index++] = (Temp >> 8) & 0xff;
    //Blue
	Temp = IIC_Read2Reg(0x13);
	ColorData[index++] = Temp & 0xff;
	ColorData[index++] = (Temp >> 8) & 0xff;
	Red = ((unsigned int)(ColorData[1] & 0xff) << 8 | (ColorData[0] & 0xff)); //2.06
	Green = ((unsigned int)(ColorData[3] & 0xff) << 8 | (ColorData[2] & 0xff));
	Blue = ((unsigned int)(ColorData[5] & 0xff) << 8 | (ColorData[4] & 0xff));



	if (Red > 4500) Red = 4500;    
	if (Green > 7600) Green = 7600;
	if (Blue > 4600) Blue = 4600;
	val_red =   Red / 17;
	val_blue = Blue / 18;
	val_green = Green / 29;


	if (val_red > val_green && val_red > val_blue)
	{
		val_red = 255;
		val_green = 0;
		val_blue = 0;
	
	}
	else if (val_green > val_red && val_green > val_blue)
	{

		val_green = 255;
		val_red = 0;
		val_blue = 0;
	
	}
	else if (val_blue > val_red && val_blue >  val_green)
	{
		val_blue = 255;
		val_red = 0;
		val_green = 0;
	}

	printk("R:%d, G:%d, B:%d\n", Red, Green, Blue);  
	printk("val_red:%d, val_green:%d, val_blue:%d\n", val_red, val_green, val_blue);
  //delay(1000);
}


void TM1650_init(void) {
    // Initialize display with maximum brightness
    TM1650_writeI2C(DISPLAY_CMD_ADDR, 0x01 | (7 << 4));  // Display ON, brightness level 7
}

void TM1650_writeI2C(uint8_t addr, uint8_t data) {
	int ret;
	switch (addr) {
		case DISPLAY_CMD_ADDR:
			ret = i2c_write_dt(&dev_i2c_led_cmd, &data, sizeof(data));
			break;
		case DISPLAY_DATA_ADDR1:
			ret = i2c_write_dt(&dev_i2c_led_data1, &data, sizeof(data));
			break;
		case DISPLAY_DATA_ADDR2:
			ret = i2c_write_dt(&dev_i2c_led_data2, &data, sizeof(data));
			break;
		case DISPLAY_DATA_ADDR3:
			ret = i2c_write_dt(&dev_i2c_led_data3, &data, sizeof(data));
			break;
		case DISPLAY_DATA_ADDR4:
			ret = i2c_write_dt(&dev_i2c_led_data4, &data, sizeof(data));
			break;
	}
}

void TM1650_setBrightness(uint8_t brightness) {
    if (brightness > 7) brightness = 7;
    TM1650_writeI2C(DISPLAY_CMD_ADDR, 0x01 | (brightness << 4));
}

void TM1650_displayDigit(uint8_t position, uint8_t digit) {
    uint8_t addr;
    
    // Select address based on position
    switch(position) {
        case 0: addr = DISPLAY_DATA_ADDR1; break;
        case 1: addr = DISPLAY_DATA_ADDR2; break;
        case 2: addr = DISPLAY_DATA_ADDR3; break;
        case 3: addr = DISPLAY_DATA_ADDR4; break;
        default: return; // Invalid position
    }
    
    if(digit > 15) digit = 0; // Limit to valid range (0-F)
    TM1650_writeI2C(addr, digitPatterns[digit]);
}

void TM1650_displayTest(void) {
    uint8_t i, j;
    
    // Test 1: Count from 0-F on all digits
    for(i = 0; i <= 15; i++) {
        for(j = 0; j < 4; j++) {
            TM1650_displayDigit(j, i);
        }
        k_sleep(K_MSEC(500)); // Delay 500ms between numbers
    }
    
    // Test 2: Rolling pattern
    for(i = 0; i < 4; i++) {
        TM1650_writeI2C(DISPLAY_DATA_ADDR1 + i, 0x00);  // Clear all
    }
    
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 7; j++) {
            TM1650_writeI2C(DISPLAY_DATA_ADDR1 + i, (1 << j));
            k_sleep(K_MSEC(100));
        }
    }
    
    // Test 3: Brightness levels
    for(i = 0; i < 4; i++) {
        TM1650_displayDigit(i, 8);  // Display "8" on all digits
    }
    
    for(i = 0; i <= 7; i++) {
        TM1650_setBrightness(i);
        k_sleep(K_MSEC(500));
    }
}