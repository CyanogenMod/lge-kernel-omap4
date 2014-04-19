#include <mach/aat2862.h>

#define MAX_BRIGHTNESS 		0x1f	// 0001 1111 =   0.48mA
#define DEFAULT_BRIGHTNESS 	0x08	// 0000 1000 = 22.26mA

extern int aat2862_write_byte(struct i2c_client *client , unsigned char Reg, unsigned char Val)
{
	int ret;
	struct aat2862data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,Reg,Val);
	mutex_unlock(&data->update_lock);
	return ret;
}

extern int aat2862_read_byte(struct i2c_client *client, unsigned char Reg)
{
	int ret;
	struct aat2862data *data= i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_read_byte_data(client, Reg);
	mutex_unlock(&data->update_lock);
	return ret;
}

extern int aat2862_init_client(struct i2c_client *client)
{
	int val;
	/////////////LDO CONTROL/////////////
	aat2862_write_byte(client, LDO_A_B_OUTPUT_VOLTAGE_CONT_REG,0x4C); // LDOA 1.8V_LCD_IOVCC(0100), LDOC 3.0V_LCD_VCC_VCI (1100) 
	aat2862_write_byte(client, LDO_C_D_OUTPUT_VOLTAGE_CONT_REG,0x4C); // LDOC 1.8V_TEMP_OUT (1100), LDOD 3.0V_TEMP_OUT (1010) 
	aat2862_write_byte(client, LDO_ENABLE_CONTORL_REG,         0x0F); //Enable all LDOs 
	//////BACKLIGHT CURRENT CONTROL//////
	val = 0xE0 | DEFAULT_BRIGHTNESS;
	aat2862_write_byte(client,MAIN_CURRENT_CONTROL_REG,val);
	val = 0x60 | DEFAULT_BRIGHTNESS;
	aat2862_write_byte(client, SUB_CURRENT_CONTROL_REG,val);
	aat2862_write_byte(client,AUX1_CURRENT_CONTROL_REG,val);
	aat2862_write_byte(client,AUX2_CURRENT_CONTROL_REG,val);
	return 0;
}

EXPORT_SYMBOL(aat2862_write_byte);
EXPORT_SYMBOL(aat2862_read_byte);
EXPORT_SYMBOL(aat2862_init_client);


MODULE_AUTHOR("LG Electronics (jaekyung.oh@lge.com)");
MODULE_DESCRIPTION("Multi Display LED driver");
MODULE_LICENSE("GPL");
