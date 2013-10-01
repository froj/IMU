

#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>


u16 adcVals[3];
u16 adcVal;
u16 adcOldVal;
u16 buffer0[2];
u16 buffer1[2];
int jerk;


static void clock_setup(void);
static void gpio_setup(void);
static void adc_setup(void);
static void setup_dma(u16* buf0, u16* buf1);
static void setup_usart(void);
void send_data(void);

static void clock_setup(){
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    /* enable ADC1 clock */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    /* Clock for the ADC pins */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    /* enable clock for LEDs */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    /* USART 3 */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
    /* DMA for ADC */
    //rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN); 
}

static void gpio_setup(){
    /* ADC */
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
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
    adc_disable_scan_mode(ADC1);
    adc_disable_eoc_interrupt_injected(ADC1);
    adc_disable_awd_interrupt(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_disable_external_trigger_injected(ADC1);
    adc_set_right_aligned(ADC1);
    adc_disable_temperature_sensor();


    /* Setup for DMA mode */
    //adc_enable_dma(ADC1);
    adc_disable_dma(ADC1);

    adc_power_on(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    u8 channels[] = {ADC_CHANNEL8, ADC_CHANNEL9}; 
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_set_sample_time(ADC1, ADC_CHANNEL8, ADC_SMPR_SMP_239DOT5CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL9, ADC_SMPR_SMP_239DOT5CYC);





    /* enable adc interrupts */
    adc_eoc_after_each(ADC1);
    nvic_enable_irq(NVIC_ADC_IRQ);
    ADC_SR(ADC1) &= ~ADC_SR_EOC;
    adc_enable_eoc_interrupt(ADC1);
    adc_clear_overrun_flag(ADC1);
    adc_enable_overrun_interrupt(ADC1);

    adc_start_conversion_regular(ADC1);
}

static void setup_dma(u16* buf0, u16* buf1){
    
    dma_stream_reset(DMA2, DMA_STREAM0);
    dma_set_peripheral_address(DMA2, DMA_STREAM0, (u32)&ADC_DR(ADC1));

    /* Double buffer mode -> 2 buffers */
    dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);
    dma_set_memory_address(DMA2, DMA_STREAM0, (u32)buf0);
    dma_set_memory_address_1(DMA2, DMA_STREAM0, (u32)buf1);

    dma_set_number_of_data(DMA2, DMA_STREAM0, 2);
    dma_set_dma_flow_control(DMA2, DMA_STREAM0);

    dma_channel_select(DMA2, DMA_STREAM0, 0);
    /* problem with the included defines. lolwut? */
    //dma_set_priority(DMA2, DMA_STREAM0, DMA_SCR_PL_HIGH);
    //dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SCR_PSIZE_16BIT);
    //dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SCR_MSIZE_16BIT);
    
    dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_ISR_FLAGS);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);

    dma_enable_stream(DMA2, DMA_STREAM0);

    /*
    bool dma_get_interrupt_flag(u32 dma, u8 stream, u32 interrupt);
    void dma_enable_transfer_error_interrupt(u32 dma, u8 stream);
    void dma_disable_transfer_error_interrupt(u32 dma, u8 stream);
    void dma_enable_half_transfer_interrupt(u32 dma, u8 stream);
    void dma_disable_half_transfer_interrupt(u32 dma, u8 stream);
    void dma_disable_transfer_complete_interrupt(u32 dma, u8 stream);
    void dma_enable_direct_mode_error_interrupt(u32 dma, u8 stream);
    void dma_disable_direct_mode_error_interrupt(u32 dma, u8 stream);
    void dma_enable_fifo_error_interrupt(u32 dma, u8 stream);
    void dma_disable_fifo_error_interrupt(u32 dma, u8 stream);
    
    void dma_set_transfer_mode(u32 dma, u8 stream, u32 direction);
    void dma_enable_memory_increment_mode(u32 dma, u8 stream);
    void dma_disable_memory_increment_mode(u32 dma, u8 channel);
    void dma_enable_peripheral_increment_mode(u32 dma, u8 stream);
    void dma_disable_peripheral_increment_mode(u32 dma, u8 channel);
    void dma_enable_fixed_peripheral_increment_mode(u32 dma, u8 stream);
    void dma_enable_circular_mode(u32 dma, u8 stream);
    void dma_set_memory_burst(u32 dma, u8 stream, u32 burst);
    void dma_set_peripheral_burst(u32 dma, u8 stream, u32 burst);
    void dma_set_initial_target(u32 dma, u8 stream, u8 memory);
    u8 dma_get_target(u32 dma, u8 stream);
    
    u32 dma_fifo_status(u32 dma, u8 stream);
    void dma_enable_direct_mode(u32 dma, u8 stream);
    void dma_enable_fifo_mode(u32 dma, u8 stream);
    void dma_set_fifo_threshold(u32 dma, u8 stream, u32 threshold);
    void dma_disable_stream(u32 dma, u8 stream);
    */
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
    setup_dma(buffer0, buffer1);
    adc_setup();
    setup_usart();


    while(42){
        gpio_clear(GPIOD, GPIO13);
        gpio_clear(GPIOD, GPIO14);

        send_data();
    }
}

void send_data(){
    usart_send_blocking(USART3, 'Z');
    usart_send_blocking(USART3, ':');
    usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, (u8)(adcVals[0] / 10000) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[0] / 1000 % 10) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[0] / 100 % 10) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[0] / 10 % 10) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[0] % 10) + '0');
    usart_send_blocking(USART3, '\n');
    usart_send_blocking(USART3, 'X');
    usart_send_blocking(USART3, ':');
    usart_send_blocking(USART3, ' ');
    usart_send_blocking(USART3, (u8)(adcVals[1] / 10000) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[1] / 1000 % 10) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[1] / 100 % 10) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[1] / 10 % 10) + '0');
    usart_send_blocking(USART3, (u8)(adcVals[1] % 10) + '0');
    usart_send_blocking(USART3, '\n');
    
}

void adc_isr(){
    static int i = 0;

    if(adc_get_overrun_flag(ADC1)){     /* has a overrun occured? */
        /* recover */
        adc_clear_overrun_flag(ADC1);
        adc_start_conversion_regular(ADC1);
        gpio_set(GPIOD, GPIO12);
    }else{      /* sequence finished? */
        adcOldVal = adcVal;
        adcVal = adc_read_regular(ADC1);
        jerk = adcVal - adcOldVal;

        adcVals[i] = adcVal;

        if(i == 0){
            i = 1;
            gpio_set(GPIOD, GPIO13);
        }
        else if(i == 1){
            i = 0;
            gpio_set(GPIOD, GPIO14);
        }
    }
}


/* DMA interrupt from the ADC's stream */
void dma2_stream0_isr(){
    u8 j;
    
    /* We wants the transfer complete interrupt, Lebowsky! */
    if(DMA2_LISR & DMA_LISR_TCIF0){
        /* Buffer 1 has been filled */
//        if(DMA_SCR(DMA2, DMA_STREAM0) & DMA_SCR_CT){
//            for(j = 0; j < 2; j++){
//                adcVals[j] = buffer1[j];
//            }
//        /* Buffer 0 has been filled */
//        }else{
//            for(j = 0; j < 2; j++){
//                adcVals[j] = buffer0[j];
//            }
//        }
    }
}
