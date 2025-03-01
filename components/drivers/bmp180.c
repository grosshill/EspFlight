#include "bmp180.h"
#include "i2c_manager.h"
#include "math.h"

bmp_colab_params_t bmp180_init(i2c_master_dev_handle_t dev_handle)
{
    bmp_colab_params_t bmp_params;
    uint8_t data[22];
    i2c_read(dev_handle, BMP180_COLAB_START, data, 22);

    bmp_params.AC1 = (short)((data[0] << 8) | data[1]);
    bmp_params.AC2 = (short)((data[2] << 8) | data[3]);
    bmp_params.AC3 = (short)((data[4] << 8) | data[5]);
    bmp_params.AC4 = (unsigned short)((data[6] << 8) | data[7]);
    bmp_params.AC5 = (unsigned short)((data[8] << 8) | data[9]);
    bmp_params.AC6 = (unsigned short)((data[10] << 8) | data[11]);
    bmp_params.B1 = (short)((data[12] << 8) | data[13]);
    bmp_params.B2 = (short)((data[14] << 8) | data[15]);
    bmp_params.MB = (short)((data[16] << 8) | data[17]);
    bmp_params.MC = (short)((data[18] << 8) | data[19]);
    bmp_params.MD = (short)((data[20] << 8) | data[21]);
    printf("%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n", bmp_params.AC1, bmp_params.AC2, bmp_params.AC3, bmp_params.AC4, bmp_params.AC5, bmp_params.AC6, bmp_params.B1, bmp_params.B2, bmp_params.MB, bmp_params.MC, bmp_params.MD);
    return bmp_params;
}

int32_t bmp180_read(i2c_master_dev_handle_t dev_handle, bmp_colab_params_t colab)
{
    uint8_t data = BMP180_CTRL_TEMP;

    i2c_write(dev_handle, BMP180_CTRL_ADDR, &data, 1);

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t temp[3];
    int32_t tep;
    int32_t hei;

    i2c_read(dev_handle, BMP180_TEMP_START, temp, 2);
    tep = (int32_t)((temp[0] << 8) + temp[1]);

    data = BMP180_CTRL_OSS;

    i2c_write(dev_handle, BMP180_CTRL_ADDR, &data, 1);

    vTaskDelay(pdTICKS_TO_MS(10));

    i2c_read(dev_handle, BMP180_TEMP_START, temp, 3);

    hei = (int32_t)((temp[0] << 16 | temp[1] << 8 | temp[2]) >> 5);
    // hei = (int32_t)(((temp[0] << 16) + (temp[1] << 8) + temp[2]) >> 8);

    int32_t UT = tep;
    int32_t UP = hei;
    bmp_colab_params_t bmp_param = colab;

    int32_t X1 = (UT-bmp_param.AC6)* bmp_param.AC5 >> 15;
	
    int32_t X2 = (bmp_param.MC << 11) / (X1 + bmp_param.MD);

    int32_t T = (X1 + X2 +8) >> 4 ;

    int32_t B6 = X1 + X2 - 4000;
		
    X1 = (B6 * B6 >> 12) * bmp_param.B2 >> 11;
    
    X2 = bmp_param.AC2 * B6 >> 11;
    
    int32_t X3 = X1 + X2;
    
    int32_t B3 = ((((bmp_param.AC1 << 2) + X3) << 3) + 2) >> 2;

    X1 = bmp_param.AC3 * B6 >> 13;
    
    X2 = (B6 * B6 >> 12) * bmp_param.B1 >> 16;
    
    X3 = (X1 + X2 + 2) >> 2; 
    
    uint32_t B4 = bmp_param.AC4 * (uint32_t)(X3 + 32768) >> 15;

    uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> 3);
    
    int32_t p;
    if(B7 < 0x80000000){
            p = (B7 << 1) / B4;  
    }else{
            p = B7/B4 << 1;
    }
    
    X1 = (p >> 8) * (p >> 8);
    
    X1 = (X1 * 3038) >> 16;
    
    X2 = (-7375 * p) >> 16;
    
    p = p + ((X1 + X2 + 3791) >> 4);



    // int32_t x1 = (tep - colab.AC6) * colab.AC5 >> 15;

    // int32_t x2 = (colab.MC << 11) / (x1 + colab.MD);

    // int32_t t = (x1 + x2 + 8) >> 4;

    // int32_t b6 = x1 + x2 - 4000;

    // x1 = (b6 * b6 >> 12) * colab.B2 >> 11;

    // x2 = colab.AC2 * b6 >> 11;

    // int32_t x3 = x1 + x2;

    // int32_t b3 = (((colab.AC1 << 2) + x3) + 2) >> 2;

    // x1 = colab.AC3 * b6 >> 13;

    // x2 = (b6 * b6 >> 12) * colab.B1 >> 16;

    // x3 = (x1 + x2 + 2) >> 2;

    // uint32_t b4 = colab.AC4 * (uint32_t)(x3 + 32768) >> 15;

    // uint32_t b7 = ((uint32_t)hei - b3) * 50000;

    // int32_t p;
    // if(b7 < 0x80000000)
    // {
    //     p = (b7 << 1) / b4;
    // }
    // else
    // {
    //     p = b7 / b4 << 1;
    // }

    // x1 = (p >> 8) * (p >> 8);
    // x1 = (x1 * 3038) >> 16;
    // x2 = (-7375 * p) >> 16;

    // p = p + ((x1 + x2 + 3791) >> 4);

    float height = 44330 * (1 - pow(p / 101325.0f, 0.190263));

    ESP_LOGI("BMP180", "temperature: %ld, pressure: %ld, height: %.10f", T, p, height);
    return hei;
}
