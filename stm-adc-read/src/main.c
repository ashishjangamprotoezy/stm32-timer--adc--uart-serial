

#include "stm32f401xe.h"

/* ================= Globals ================= */
volatile uint16_t adc_buffer;
volatile uint8_t dma_done = 0;

/* ================= Prototypes ================= */
void gpio_init(void);
void tim2_init_100hz_trgo(void);
void adc1_init_dma(void);
void uart2_init(void);
void uart2_send_uint(uint16_t val);

/* ================= MAIN ================= */
int main(void)
{
    gpio_init();
    uart2_init();
    adc1_init_dma();
    tim2_init_100hz_trgo();

    while (1)
    {
        //uart2_send_uint(1234);
         //GPIOA->ODR ^= (1U << 5);
        //for (volatile int i = 0; i < 300000; i++);
        
        if (dma_done)
        {
            dma_done = 0;
            uart2_send_uint(adc_buffer);
        }

        __WFI();
    }
}

/* ================= GPIO ================= */
void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA5 LED */
    //GPIOA->MODER |= (1U << (5 * 2));
    GPIOA->MODER &= ~(3U << (5 * 2));
GPIOA->MODER |=  (1U << (5 * 2));
    /* PA0 ADC */
    GPIOA->MODER |= (3U << (0 * 2));

    /* PA2 UART TX */
    GPIOA->MODER |= (2U << (2 * 2));
    GPIOA->AFR[0] |= (7U << (2 * 4));
}
void tim2_init_100hz_trgo(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 8400 - 1;   // 10 kHz
    TIM2->ARR = 100 - 1;    // 100 Hz

    TIM2->CR1 = 0;
    TIM2->CR2 = 0;

    TIM2->CR2 |= TIM_CR2_MMS_1;   // Update event as TRGO

    TIM2->EGR |= TIM_EGR_UG;      // 🔥 FORCE first update event
    TIM2->CR1 |= TIM_CR1_CEN;     // Enable timer
}
void adc1_init_dma(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* ADC common clock = PCLK2 / 4 */
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    /* ---------- DMA ---------- */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);

    DMA2->LIFCR =
        DMA_LIFCR_CTCIF0 |
        DMA_LIFCR_CHTIF0 |
        DMA_LIFCR_CTEIF0 |
        DMA_LIFCR_CDMEIF0 |
        DMA_LIFCR_CFEIF0;

    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)&adc_buffer;
    DMA2_Stream0->NDTR = 1;

    DMA2_Stream0->CR =
        DMA_SxCR_PSIZE_0 |
        DMA_SxCR_MSIZE_0 |
        DMA_SxCR_CIRC |
        DMA_SxCR_TCIE;

    NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    DMA2_Stream0->CR |= DMA_SxCR_EN;

    /* ---------- ADC ---------- */
    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;                // Channel 0 (PA0)
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;

    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_EXTEN_0;
    ADC1->CR2 |= (4 << ADC_CR2_EXTSEL_Pos); // TIM2 TRGO

    ADC1->CR2 |= ADC_CR2_ADON;

    ADC1->CR2 |= ADC_CR2_SWSTART;   // 🔥 REQUIRED LINE (THE FIX)
}

/* ================= DMA ISR ================= */
void DMA2_Stream0_IRQHandler(void)
{
 //GPIOA->ODR &= ~(1U << 5); 
    if (DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
        dma_done = 1;

        GPIOA->ODR ^= (1U << 5);   // LED toggle on DMA complete
    }
}

/* ================= UART ================= */
void uart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

   // USART2->BRR = 0x1117;   // 9600 baud @ 84MHz
   USART2->BRR = 0x0683;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void uart2_send_uint(uint16_t val)
{
    char buf[10];
    int i = 0;

    if (val == 0) buf[i++] = '0';
    else
    {
        char tmp[10];
        int j = 0;
        while (val)
        {
            tmp[j++] = (val % 10) + '0';
            val /= 10;
        }
        while (j--) buf[i++] = tmp[j];
    }

    buf[i++] = '\r';
    buf[i++] = '\n';

    for (int k = 0; k < i; k++)
    {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = buf[k];
    }
}
