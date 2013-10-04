

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

u16 adcVals[3];
int accVals[3];
uint16_t buffer0[3];
uint16_t buffer1[3];


static void clock_setup(void);
static void gpio_setup(void);
static void adc_setup(void);
static void setup_dma(uint16_t* buf0, uint16_t* buf1);
static void setup_dma_uart3(void);
static void setup_usart(void);
void start_usart3_tx(uint32_t buf[], int length);
void start_usart3_rx(uint8_t buf[], int length, bool circular);
void stop_usart3_rx(void);
void send_data(void);

static void clock_setup(){
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    /* enable ADC1 clock */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    /* Clock for the ADC pins */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    /* enable clock for LEDs */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    /* USART 3 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
    /* DMA for ADC */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN); 
    /* DMA for USART */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA1EN); 
}

static void gpio_setup(){
    /* ADC */
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    /* LEDs */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

static void adc_setup(){

    /* init all the unneeded shit */
    adc_disable_analog_watchdog_regular(ADC1);
    adc_disable_analog_watchdog_injected(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);
    adc_disable_discontinuous_mode_injected(ADC1);
    adc_disable_automatic_injected_group_conversion(ADC1);
    adc_disable_eoc_interrupt_injected(ADC1);
    adc_disable_awd_interrupt(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_disable_external_trigger_injected(ADC1);
    adc_set_right_aligned(ADC1);
    adc_disable_temperature_sensor();


    /* Setup for DMA mode */
    adc_enable_dma(ADC1);
    adc_set_dma_continue(ADC1);
    //adc_disable_dma(ADC1);

    adc_power_on(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    u8 channels[] = {ADC_CHANNEL9, ADC_CHANNEL11, ADC_CHANNEL8}; 
    adc_set_regular_sequence(ADC1, 3, channels);
    adc_set_sample_time(ADC1, ADC_CHANNEL8, ADC_SMPR_SMP_480CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL9, ADC_SMPR_SMP_480CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL11, ADC_SMPR_SMP_480CYC);


    /* enable adc interrupts */
    nvic_enable_irq(NVIC_ADC_IRQ);
    //adc_eoc_after_each(ADC1);
    //ADC_SR(ADC1) &= ~ADC_SR_EOC;
    //adc_enable_eoc_interrupt(ADC1);
    adc_clear_overrun_flag(ADC1);
    //adc_enable_overrun_interrupt(ADC1);

    adc_start_conversion_regular(ADC1);
}

static void setup_dma_uart3(){

    dma_stream_reset(DMA1, DMA_STREAM1); /* Channel 4; RX */
    dma_stream_reset(DMA1, DMA_STREAM4); /* Channel 7; TX */

    /* Peripheral address */
    dma_set_peripheral_address(DMA1, DMA_STREAM1, (u32)&USART3_DR);
    dma_set_peripheral_address(DMA1, DMA_STREAM4, (u32)&USART3_DR);

    /* Low level shit */
    dma_set_dma_flow_control(DMA1, DMA_STREAM1);
    dma_set_dma_flow_control(DMA1, DMA_STREAM4);

    dma_channel_select(DMA1, DMA_STREAM1, DMA_SxCR_CHSEL_4);
    dma_channel_select(DMA1, DMA_STREAM4, DMA_SxCR_CHSEL_7);

    dma_set_priority(DMA1, DMA_STREAM1, DMA_SxCR_PL_HIGH);
    dma_set_priority(DMA1, DMA_STREAM4, DMA_SxCR_PL_VERY_HIGH);

    /* Memory stuff */
    dma_set_peripheral_size(DMA1, DMA_STREAM1, DMA_SxCR_PSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM4, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_STREAM1, DMA_SxCR_MSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_STREAM4, DMA_SxCR_MSIZE_8BIT);

    dma_enable_memory_increment_mode(DMA1, DMA_STREAM1);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM4);
    
    /* Interrupts */
    dma_clear_interrupt_flags(DMA1, DMA_STREAM1, DMA_ISR_FLAGS);
    dma_clear_interrupt_flags(DMA1, DMA_STREAM4, DMA_ISR_FLAGS);
    nvic_enable_irq(NVIC_DMA1_STREAM1_IRQ);
    nvic_enable_irq(NVIC_DMA1_STREAM4_IRQ);
}

void start_usart3_tx(uint32_t buf[], int length){
    dma_stream_reset(DMA1, DMA_STREAM4); /* Channel 7; TX */
    dma_set_number_of_data(DMA1, DMA_STREAM4, length);
    dma_set_memory_address(DMA1, DMA_STREAM4, (u32)buf);
    usart_enable_tx_dma(USART3);
    dma_clear_interrupt_flags(DMA1, DMA_STREAM4, DMA_ISR_FLAGS);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM4);
    USART3_SR &= ~USART_SR_TC; /* clear TC bit */
    dma_enable_stream(DMA1, DMA_STREAM4);
}

void start_usart3_rx(uint8_t buf[], int length, bool circular){
    dma_stream_reset(DMA1, DMA_STREAM1); /* Channel 4; RX */
    dma_set_number_of_data(DMA1, DMA_STREAM1, length);
    dma_set_memory_address(DMA1, DMA_STREAM1, (u32)buf);

    if(circular) dma_enable_circular_mode(DMA1, DMA_STREAM1);
    else DMA_SCR(DMA1, DMA_STREAM1) &= ~DMA_SxCR_CIRC;

    usart_enable_rx_dma(USART3);
    dma_clear_interrupt_flags(DMA1, DMA_STREAM1, DMA_ISR_FLAGS);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM1);
    dma_enable_stream(DMA1, DMA_STREAM1);
}

void stop_usart3_rx(){
    dma_disable_transfer_complete_interrupt(DMA1, DMA_STREAM1);
    usart_disable_rx_dma(USART3);
    dma_disable_stream(DMA1, DMA_STREAM1);
    DMA_SCR(DMA1, DMA_STREAM1) &= ~DMA_SxCR_CIRC;
}

static void setup_dma(uint16_t* buf0, uint16_t* buf1){
    
    dma_stream_reset(DMA2, DMA_STREAM0);
    dma_set_peripheral_address(DMA2, DMA_STREAM0, (u32)&ADC_DR(ADC1));

    /* Double buffer mode -> 2 buffers */
    dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);
    dma_set_memory_address(DMA2, DMA_STREAM0, (u32)buf0);
    dma_set_memory_address_1(DMA2, DMA_STREAM0, (u32)buf1);

    dma_set_number_of_data(DMA2, DMA_STREAM0, 3);
    dma_set_dma_flow_control(DMA2, DMA_STREAM0);

    dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
    dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_HIGH);
    dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    
    dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_ISR_FLAGS);
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);

    dma_enable_stream(DMA2, DMA_STREAM0);
}

static void setup_usart(void){
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
            /* USART3_TX */
            GPIO10 |
            /* USART3_RX */
            GPIO11);

    gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11);

    usart_set_baudrate(USART3, 9600);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    usart_enable(USART3);
}



int main(void){
    clock_setup();
    gpio_setup();
    adc_setup();
    setup_dma(buffer0, buffer1);
    setup_usart();


    while(42){
        gpio_clear(GPIOD, GPIO13);
        gpio_clear(GPIOD, GPIO14);

        send_data();
    }
}

int cheap_abs(int p){
    if(p < 0) return -p;
    else return p;
}

void send_data(){
    if(accVals[0] < 0) usart_send_blocking(USART3, '-');
    else usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[0]) / 1000 % 10) + '0');
    usart_send_blocking(USART3, '.');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[0]) / 100 % 10) + '0');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[0]) / 10 % 10) + '0');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[0]) % 10) + '0');
    usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, ' ');
    if(accVals[1] < 0) usart_send_blocking(USART3, '-');
    else usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[1]) / 1000 % 10) + '0');
    usart_send_blocking(USART3, '.');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[1]) / 100 % 10) + '0');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[1]) / 10 % 10) + '0');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[1]) % 10) + '0');
    usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, ' ');
    if(accVals[2] < 0) usart_send_blocking(USART3, '-');
    else usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[2]) / 1000 % 10) + '0');
    usart_send_blocking(USART3, '.');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[2]) / 100 % 10) + '0');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[2]) / 10 % 10) + '0');
    usart_send_blocking(USART3, (u8)(cheap_abs(accVals[2]) % 10) + '0');
    usart_send_blocking(USART3, '\n');
}

void adc_isr(){
    //static int i = 0;

    if(adc_get_overrun_flag(ADC1)){     /* has a overrun occured? */
        /* recover */
        adc_clear_overrun_flag(ADC1);
        adc_start_conversion_regular(ADC1);
    }else{      /* sequence finished? */
        if(adc_eoc(ADC1)) gpio_set(GPIOD, GPIO15);
        else gpio_set(GPIOD, GPIO12);
    }
}


/* DMA interrupt from the ADC's stream */
void dma2_stream0_isr(){
    /* We wants the transfer complete interrupt, Lebowsky! */
    if(DMA2_LISR & DMA_LISR_TCIF0){
        /* Buffer 1 has been filled */
        if(DMA_SCR(DMA2, DMA_STREAM0) & DMA_SxCR_CT){
            dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_ISR_FLAGS);
            accVals[0] = (buffer1[0] - 1736) * 1000 / 345;
            accVals[1] = (buffer1[1] - 1724) * 1000 / 345;
            accVals[2] = (buffer1[2] - 1752) * 1000 / 345;
            gpio_set(GPIOD, GPIO13);
        /* Buffer 0 has been filled */
        }else{
            dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_ISR_FLAGS);
            accVals[0] = (buffer0[0] - 1736) * 1000 / 345;
            accVals[1] = (buffer0[1] - 1724) * 1000 / 345;
            accVals[2] = (buffer0[2] - 1752) * 1000 / 345;
            gpio_set(GPIOD, GPIO14);
        }
    }
}


/* DMA interrupt for USART3 RX */
void dma1_stream1_isr(){
    /* We wants the transfer complete interrupt, Lebowsky! */
    if(DMA1_LISR & DMA_LISR_TCIF1){
        
    }
}


/* DMA interrupt for USART3 TX */
void dma1_stream4_isr(){
    /* We wants the transfer complete interrupt, Lebowsky! */
    if(DMA1_HISR & DMA_HISR_TCIF4){
        dma_disable_transfer_complete_interrupt(DMA1, DMA_STREAM4);
        usart_disable_tx_dma(USART3);
        dma_disable_stream(DMA1, DMA_STREAM4);
    }
}
