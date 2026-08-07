#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("\n--- STM32 Quadrature Encoder Test (TIM3) ---");

  // Enable clocks for GPIOA and TIM3
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  // Configure PA6 and PA7 as alternate function push-pull (AF2 for TIM3)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Utilizing the external 10k pull-ups mentioned in your diagram
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // AF2 maps PA6/PA7 to TIM3
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure TIM3 in Encoder Mode (TI1 and TI2 connected)
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_HandleTypeDef htim3 = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535; // Max 16-bit counter range
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Count on both edge transitions
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0x0F; // Added digital filter to match your RC noise reduction

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0x0F;

  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Serial.println("Initialization Error!");
    while (1);
  }

  // Start the encoder interface in interrupt or counter mode
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void loop() {
  // Read the current 16-bit encoder count from TIM3
  int16_t count = (int16_t)TIM3->CNT;

  Serial.print("Encoder Count: ");
  Serial.println(count);

  delay(100);
}
