#include "mbed.h"
#include "BSP.h"

extern UART_HandleTypeDef hDiscoUart;
#ifdef __GNUC__
/* With GCC/RAISONANCE, small msg_info (option LD Linker->Libraries->Small msg_info
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
#define TIMESTEP            0.05
#define SCALE_MULTIPLIER    0.045
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};
int   AccOffset[3] = {};
float GyroOffset[3] = {};

Serial pc(USBTX, USBRX);


// main() runs in its own thread in the OS
int main()
{
  pc.baud(9600);
  HAL_Init();

  /* Configure the System clock to have a frequency of 80 MHz */
  // SystemClock_Config();
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1; 
  hDiscoUart.Init.BaudRate = 9600;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);

  BSP_ACCELERO_Init();    
  BSP_GYRO_Init();
  calibrate();

  Gyro_Test();
  // Accelero_Test();

}

void Accelero_Test(void)
{
  uint32_t ret = 0;
  pc.printf("\n***************************************************************\n");
  pc.printf("\n******************* Accelerometer Test ************************\n");
  pc.printf("\n***************************************************************\n\n");
  BSP_ACCELERO_Init();
  pc.printf("\n*** Type n or N to get a first accelero data ***\n\n");
  pc.printf("\n*** Type q or Q to quit accelerometer Test ***\n\n");
  while(1)
  {
    pc.printf("\n*** This is a new data ***\n\n");
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    for (int i = 0; i < 3; ++i) {
      pDataXYZ[i] = pDataXYZ[i] - AccOffset[i];
    }
    pc.printf("ACCELERO_X = %d \n", pDataXYZ[0]);
    pc.printf("ACCELERO_Y = %d \n", pDataXYZ[1]);
    pc.printf("ACCELERO_Z = %d \n", pDataXYZ[2]);
    wait(TIMESTEP);

    //   BSP_ACCELERO_DeInit();
    //   pc.printf("\n*** End of Accelerometer Test ***\n\n");
    //   return;  
  }    
}

void Gyro_Test(void)
{
  // uint32_t ret = 0;
  pc.printf("\n***************************************************************\n");
  pc.printf("\n************************* Gyro Test ***************************\n");
  pc.printf("\n***************************************************************\n\n");
  
  BSP_GYRO_Init();
  pc.printf("\n*** Type n or N to get a first gyro data ***\n\n");
  pc.printf("\n*** Type q or Q to quit Gyro Test ***\n\n");
  while(1)
  {
    pc.printf("\n*** This is a new data ***\n\n");
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    for (int i = 0; i < 3; ++i) {
      pGyroDataXYZ[i] = (pGyroDataXYZ[i] - GyroOffset[i]) * SCALE_MULTIPLIER;
    }
    pc.printf("GYRO_X = %.2f \n", pGyroDataXYZ[0]);
    pc.printf("GYRO_Y = %.2f \n", pGyroDataXYZ[1]);
    pc.printf("GYRO_Z = %.2f \n", pGyroDataXYZ[2]);
    wait(TIMESTEP);
    //   BSP_GYRO_DeInit();
    //   printf("\n*** End of Gyro Test ***\n\n");
    //   return;  
  }    
}


void calibrate() {
    int _sample_num = 0;
    pc.printf("calibrate...\n");

    while (_sample_num < 2000) {
      _sample_num++;
      BSP_GYRO_GetXYZ(pGyroDataXYZ);
      BSP_ACCELERO_AccGetXYZ(pDataXYZ);
      for (int i = 0; i < 3; ++i) {
          GyroOffset[i] += pGyroDataXYZ[i];
          AccOffset[i] += pDataXYZ[i];
      }
      wait(0.0005);
    }

    // for (int i = 0; i < 3; ++i)
    //     pc.printf("%.2f ", GyroOffset[i]);
    // pc.printf("\n");

    for (int i = 0; i < 3; ++i) {
        GyroOffset[i] /= _sample_num;
        AccOffset[i] /= _sample_num;
    }

    // for (int i = 0; i < 3; ++i)
    //     pc.printf("%.2f", GyroOffset[i]);
    // pc.printf("\n");

    pc.printf("Done calibration\n");
    _sample_num = 0;
}

