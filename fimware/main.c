#include <stdio.h>

#include "ch32fun.h"
#include "color_utilities.h"

#define RANDOM_STRENGTH 1
#include "lib_rand.h"

// use defines to make more meaningful names for our GPIO pins

#ifndef array_size
#define array_size( x ) ( sizeof( x ) / sizeof( ( x )[0] ) )
#endif
#ifndef min
#define min( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif
#ifndef max
#define max( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

#define INTERNAL_VREF 1200
#define ADC_RESOLUTION 10
#define ADC_MAX ( 1 << ADC_RESOLUTION )
#define ADC_SAMPLES ( 3 )

// Feedback Resistors in 10 Ohm units
#define Rf 390
#define Rin 100
#define Rt ( Rf + Rin )

#define L0 PC1
#define L1 PC2
#define L2 PC3
#define L3 PC4
#define L4 PC6
#define L5 PC7
#define L6 PD4

uint8_t pins[] = { L0, L1, L2, L3, L4, L5, L6 };

typedef union
{
	uint32_t abgr;
	struct
	{
		uint8_t r, b, g, a; // got g and b backwards
	};
} color_t;

#define COLOUR_1 0x1020FF // Red
#define COLOUR_2 0x20FF10 // Green
#define COLOUR_3 0x10FF7F // Yellow
#define COLOUR_4 0xFF2010 // Blue
#define COLOUR_5 0xFF107F // Magenta
#define COLOUR_6 0xFFFF10 // Cyan
#define COLOUR_7 0xFFEF7F // White
#define COLOUR_8 0x10507F // Orange

typedef struct
{
	uint8_t pin_anode;
	uint8_t pin_red;
	uint8_t pin_green;
	uint8_t pin_blue;
	color_t color;
	uint8_t brightness;
	int8_t direction;
} led_t;

typedef enum
{
	LOW = 0,
	HIGH = 1,
	FLOAT = 2,
} pin_state_e;

static led_t led_lookup[] = {
	{ .pin_anode = L0, .pin_red = L3, .pin_green = L2, .pin_blue = L1 },
	{ .pin_anode = L1, .pin_red = L3, .pin_green = L2, .pin_blue = L0 },
	{ .pin_anode = L2, .pin_red = L3, .pin_green = L1, .pin_blue = L0 },
	{ .pin_anode = L3, .pin_red = L2, .pin_green = L1, .pin_blue = L0 },
	{ .pin_anode = L5, .pin_red = L2, .pin_green = L1, .pin_blue = L0 },
	{ .pin_anode = L4, .pin_red = L2, .pin_green = L1, .pin_blue = L0 },

	{ .pin_anode = L5, .pin_red = L6, .pin_green = L4, .pin_blue = L3 },
	{ .pin_anode = L3, .pin_red = L6, .pin_green = L5, .pin_blue = L4 },
	{ .pin_anode = L4, .pin_red = L6, .pin_green = L5, .pin_blue = L3 },
	{ .pin_anode = L2, .pin_red = L6, .pin_green = L5, .pin_blue = L4 },
	{ .pin_anode = L1, .pin_red = L6, .pin_green = L5, .pin_blue = L4 },
};

static const color_t colors[] = {
	{ COLOUR_1 },
	{ COLOUR_2 },
	{ COLOUR_3 },
	{ COLOUR_4 },
	{ COLOUR_5 },
	{ COLOUR_6 },
	{ COLOUR_7 },
	{ COLOUR_8 },
};
// clang-format off
// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.50 steps = 256 range = 0-255
const uint8_t gamma_lut[256] = {
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   4,   4,
     4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   7,   8,
     8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  12,  12,  12,  13,  13,  14,
    14,  15,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,  22,
    22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,  29,  30,  30,  31,  32,
    33,  33,  34,  35,  36,  36,  37,  38,  39,  40,  40,  41,  42,  43,  44,  45,
    46,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
    61,  62,  63,  64,  65,  67,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,
    80,  81,  82,  83,  85,  86,  87,  89,  90,  91,  93,  94,  95,  97,  98,  99,
   101, 102, 104, 105, 107, 108, 110, 111, 113, 114, 116, 117, 119, 121, 122, 124,
   125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 150, 151,
   153, 155, 157, 159, 161, 163, 165, 166, 168, 170, 172, 174, 176, 178, 180, 182,
   184, 186, 189, 191, 193, 195, 197, 199, 201, 204, 206, 208, 210, 212, 215, 217,
   219, 221, 224, 226, 228, 231, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255,
  };
// clang-format on

static uint16_t s_feedbackVRaw = 0;
static uint16_t s_vref = 0;
static volatile uint32_t s_systickCount = 0;


static inline color_t RandomColor( void )
{
	return colors[rand() & 7];
}

static void SysTick_Init( void );
static void ADC_Init( void );
static void PWM_Init( void );

static inline void pinSet( int pin, pin_state_e state );
static void clear( void );
static int GetVRefMillivolts( void );


#define FADE_SPEED 1

// TODO: add sleep on low power

int main()
{
	SystemInit();
	SysTick_Init();
	funAnalogInit();
	seed( SysTick->CNT + funAnalogRead( 8 ) );

	funGpioInitAll(); // Enable GPIOs
	clear();

	ADC_Init();
	PWM_Init();

	Delay_Ms( 1000 );
	printf( "Starting with clock %u MHz\n", FUNCONF_SYSTEM_CORE_CLOCK / 1000000 );


	for ( size_t i = 0; i < array_size( led_lookup ); i++ )
	{
#if 1
		const uint8_t r = ( 128 + rand() ) & 0xFF;
#else
		const uint8_t r = 127;
#endif
		led_lookup[i].brightness = r;
		led_lookup[i].direction = r & ( 1 << 7 ) ? -FADE_SPEED : FADE_SPEED;
		led_lookup[i].color = RandomColor();
		// led_lookup[i].color = (color_t){COLOUR_7}; // force white for testing
	}

	uint32_t last = s_systickCount;
	uint32_t ticks = 0;


	while ( 1 )
	{
		if ( s_systickCount - last > 16 ) // 60fps
		{
			last = s_systickCount;
			ticks++;

#if FADE_SPEED
			for ( size_t i = 0; i < array_size( led_lookup ); i++ )
			{
				if ( led_lookup[i].brightness >= 255 - led_lookup[i].direction )
				{
					led_lookup[i].direction = -FADE_SPEED;
				}
				else if ( led_lookup[i].brightness <= -led_lookup[i].direction )
				{
					led_lookup[i].direction = FADE_SPEED;
					led_lookup[i].color = RandomColor();
					led_lookup[i].brightness += rand() & 0x7;
				}
				led_lookup[i].brightness += led_lookup[i].direction;
			}
#endif
		}

		if ( ticks % 60 == 0 )
		{
			// printf( "Vbat = %d mV (%d)(%d)\n", GetVRefMillivolts(), (int)s_vref, (int)s_feedbackVRaw );
		}

		for ( size_t i = 0; i < array_size( led_lookup ); i++ )
		{
			// Delay_Ms( 10 );
			// Delay_Us( 50 );
			led_t *led = &led_lookup[i];

			color_t color = led->color;
			uint8_t b = gamma_lut[led->brightness];
			color.r = FastMultiply( color.r, b ) >> 8;
			color.g = FastMultiply( color.g, b ) >> 8;
			color.b = FastMultiply( color.b, b ) >> 8;

			if ( color.abgr != 0 )
			{
				pinSet( led->pin_red, LOW );
				pinSet( led->pin_green, LOW );
				pinSet( led->pin_blue, LOW );
				pinSet( led->pin_anode, HIGH );
			}

			for ( size_t i = 0; i < 256; i++ )
			{
				if ( color.r == 0 )
				{
					pinSet( led->pin_red, FLOAT );
				}
				else
				{
					color.r--;
				}

				if ( color.g == 0 )
				{
					pinSet( led->pin_green, FLOAT );
				}
				else
				{
					color.g--;
				}

				if ( color.b == 0 )
				{
					pinSet( led->pin_blue, FLOAT );
				}
				else
				{
					color.b--;
				}

				Delay_Us( 1 );
			}

			// Delay_Ms( 10 );
			// Delay_Us( 1000 );
			pinSet( led->pin_anode, FLOAT );
		}
	}
}

void SysTick_Handler( void ) __attribute__( ( interrupt ) );
void SysTick_Handler( void )
{
	// Set the next interrupt to be in 1/1000th of a second
	SysTick->CMP += ( FUNCONF_SYSTEM_CORE_CLOCK / 1000 );

	// Clear IRQ
	SysTick->SR = 0;

	// Update counter
	s_systickCount++;
}

void ADC1_IRQHandler( void ) __attribute__( ( interrupt ) );
void ADC1_IRQHandler( void )
{
	// Values come in reverse order.
	s_feedbackVRaw = ADC1->IDATAR1;

	s_vref = ADC1->RDATAR;

	// Acknowledge pending interrupts.
	ADC1->STATR = 0;
}

static void SysTick_Init( void )
{
	// Disable default SysTick behavior
	SysTick->CTLR = 0;

	// Enable the SysTick IRQ
	NVIC_EnableIRQ( SysTick_IRQn );

	// Set the tick interval to 1ms for normal op
	SysTick->CMP = SysTick->CNT + ( FUNCONF_SYSTEM_CORE_CLOCK / 1000 ) - 1;

	// Start at zero
	s_systickCount = 0;

	// Enable SysTick counter, IRQ, HCLK/1
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
}

/**
 * @brief  Set up the ADC for the voltage and current feedback
 * @param  None
 * @return None
 */
static void ADC_Init( void )
{
	// Configure ADC.
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1;

	// PD6 is analog input ch 6
	GPIOD->CFGLR &= ~( 0xf << ( 6 << 2 ) ); // CNF = 00: Analog, MODE = 00: Input

	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	// ADCCLK = 12 MHz => RCC_ADCPRE divide by 4
	RCC->CFGR0 &= ~RCC_ADCPRE; // Clear out the bis in case they were set
	RCC->CFGR0 |= RCC_ADCPRE_DIV4; // set it to 010xx for /4.

	ADC1->RSQR1 = 0; // 1 channels in sequence
	ADC1->RSQR2 = 0;
	// Set up 1st conversion on ch6
	// 0-9 for 8 ext inputs and two internals
	ADC1->RSQR3 = ( 8 << 0 );

	// Injection group is 8. NOTE: See note in 9.3.12 (ADC_ISQR) of TRM. The
	//  group numbers is actually 4-group numbers.
	ADC1->ISQR = ( 6 << 15 ); // ch8 as first conversion
	ADC1->ISQR |= ( 1 << 20 ); // set number of conversions to 1

	// Sampling time for channels. Careful: This has PID tuning implications.
	// Note that with 3 and 3,the full loop (and injection) runs at 138kHz.
	// set sampling time for all channels to 15 (A good middleground) ADC_SMP0_1.
	ADC1->SAMPTR2 = ( ADC_SMP0_1 << ( 3 * 0 ) ) | ( ADC_SMP0_1 << ( 3 * 1 ) ) | ( ADC_SMP0_1 << ( 3 * 2 ) ) |
	                ( ADC_SMP0_1 << ( 3 * 3 ) ) | ( ADC_SMP0_1 << ( 3 * 4 ) ) | ( ADC_SMP0_1 << ( 3 * 5 ) ) |
	                ( ADC_SMP0_1 << ( 3 * 6 ) ) | ( ADC_SMP0_1 << ( 3 * 7 ) ) | ( ADC_SMP0_1 << ( 3 * 8 ) ) |
	                ( ADC_SMP0_1 << ( 3 * 9 ) );
	ADC1->SAMPTR1 = ( ADC_SMP0_1 << ( 3 * 0 ) ) | ( ADC_SMP0_1 << ( 3 * 1 ) ) | ( ADC_SMP0_1 << ( 3 * 2 ) ) |
	                ( ADC_SMP0_1 << ( 3 * 3 ) ) | ( ADC_SMP0_1 << ( 3 * 4 ) ) | ( ADC_SMP0_1 << ( 3 * 5 ) );
	// 0:7 => 3/9/15/30/43/57/73/241 cycles
	// (4 == 43 cycles), (6 = 73 cycles)  Note these are alrady /2, so
	// setting this to 73 cycles actually makes it wait 256 total cycles @ 48MHz.

	// Turn on ADC and set rule group to sw trig
	// 0 = Use TRGO event for Timer 1 to fire ADC rule.
	ADC1->CTLR2 = ADC_ADON | ADC_JEXTTRIG | ADC_JEXTSEL | ADC_EXTTRIG;
	ADC1->CTLR2 |= ADC_EXTSEL_0 | ADC_EXTSEL_1; // Select TIM2 TRGO event
	// ADC1->CTLR2 |= ADC_JEXTSEL_0 | ADC_JEXTSEL_1; // Select TIM2 CH4 event

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while ( ADC1->CTLR2 & ADC_RSTCAL );

	// Calibrate ADC
	ADC1->CTLR2 |= ADC_CAL;
	while ( ADC1->CTLR2 & ADC_CAL );

	// enable the ADC Conversion Complete IRQ
	NVIC_EnableIRQ( ADC_IRQn );

	// ADC_JEOCIE: Enable the End-of-conversion interrupt.
	// ADC_JDISCEN | ADC_JAUTO: Force injection after rule conversion.
	// ADC_SCAN: Allow scanning.
	ADC1->CTLR1 = ADC_JEOCIE | ADC_JDISCEN | ADC_SCAN | ADC_JAUTO;
}

static void PWM_Init( void )
{
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	AFIO->PCFR1 |= GPIO_FullRemap_TIM2;

	// PD5 is T2CH4
	funPinMode( PD5, ( GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF ) );

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// CTLR1: default is up, events generated, edge align
	// SMCFGR: default clk input is CK_INT

	// Prescaler
	TIM2->PSC = 0x0001;

	// Auto Reload - sets period
	TIM2->ATRLR = 255 + 10;

	// Reload immediately
	TIM2->SWEVGR |= TIM_UG;

	// Enable CH4 output, normal polarity
	TIM2->CCER |= TIM_CC4E | TIM_CC4NP;

	// CH4 Mode is output, PWM1 (CC4S = 00, OC4M = 110)
	TIM2->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;

	// Set the Capture Compare Register value to halfway
	TIM2->CH4CVR = TIM2->ATRLR >> 1;

	// Setup TRGO for ADC.  This makes is to the ADC will trigger on timer
	// reset, so we trigger at the same position every time relative to the
	// FET turning on.
	TIM2->CTLR2 = TIM_MMS_1;

	// Enable TIM2 outputs
	TIM2->BDTR |= TIM_MOE;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
}

static inline void pinSet( int pin, pin_state_e state )
{
	if ( state == HIGH )
	{
		funPinMode( pin, GPIO_CFGLR_OUT_10Mhz_PP );
		funDigitalWrite( pin, FUN_HIGH );
	}
	else if ( state == LOW )
	{
		funPinMode( pin, GPIO_CFGLR_OUT_10Mhz_PP );
		funDigitalWrite( pin, FUN_LOW );
	}
	else
	{
		funPinMode( pin, GPIO_CFGLR_IN_FLOAT );
	}
}

static void clear( void )
{
	for ( size_t i = 0; i < array_size( pins ); i++ )
	{
		pinSet( pins[i], FLOAT ); // Set all pins to FLOAT
		led_lookup[i].color.abgr = 0;
	}
}

/**
 * @brief  Get the VRef voltage in millivolts
 * @param  None
 * @return The VRef voltage in millivolts
 */
static int GetVRefMillivolts( void )
{
	return ( ( INTERNAL_VREF * ADC_MAX ) / s_vref );
}

