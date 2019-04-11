// ***** 0. Documentation Section *****
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h" 



// ***** 2. Global Declarations Section *****

//Next parts are not needed because of  #include "tm4c123gh6pm.h" 


#define SENSOR (*((volatile unsigned long *)0x4002401C)) 
#define CAR_SIGNAL (*((volatile unsigned long *)0x400050FC)) 
#define PEDSTRIAN_SIGNAL (*((volatile unsigned long *)0x40025028))

// FSM Engine data structure
struct state
{
	
	unsigned long Output[2]; // Output = { LIGHT_CAR(PB5-PB0), LIGHT_PEDS(PF3 and PF1) }
	
	unsigned long Time; // Waitig time in a valid state/Time delay.
	
	unsigned long NextState[8]; // NextState depending on the Current_States 
	                           // and Input(SENSORS values means the states of PE2-PE0,
	                          // possible states are - 000,001,010,011,100,101,110,111 or 0,1,2,3,4,5,6,7)
	
};

typedef const struct state sTyp; // Creating a constant user define data type and store it into ROM. 

#define goWest 0 // Go for West(West traffic signal -> Red, South traffic signal -> Red,
                //Pedstrian traffic signal -> Red means PB5-PB0 = 001100 and PF3 = 0,PF1 = 1)

#define waitWest 1 // Wait for the transition from West to South/Pedstrian(West traffic signal -> Yellow, 
                  //South traffic signal -> Red, Pedstrian traffic signal -> Red means PB5-PB0 = 010100 and PF3 = 0,PF1 = 1)

#define goSouth 2 // Go for South(West traffic signal -> Red, South traffic signal -> Green,
                 //Pedstrian traffic signal -> Red means PB5-PB0 = 100001 and PF3 = 0,PF1 = 1)

#define waitSouth 3 // Wait for the transition from South to West/Pedstrian(West traffic signal -> Red,
                   // South traffic signal -> Yellow, Pedstrian traffic signal -> Red means PB5-PB0 = 100010 and PF3 = 0,PF1 = 1)

#define goWalk 4 // Go for Pedstrian(West traffic signal -> Red, South traffic signal -> Red,
                // Pedstrian traffic signal -> Green means PB5-PB0 = 100100 and PF3 = 1,PF1 = 0)

#define waitWalk 5  // Wait for the transition from Pedstrian to South/West(West traffic signal -> Red,
                   // South traffic signal -> Red, Pedstrian traffic signal -> Red(toggling) means PB5-PB0 = 100100 and PF3 = 0,PF1 = 1)


// one-to-one mapping of outputs,time delays and next states form graphical representation to data structure form.
sTyp FSM[6] = {
               {
							  {0x0C, 0x02}, 10, {goWest, goWest, waitWest, waitWest, waitWest, waitWest, waitWest, waitWest} // goWest
							 },
							 
               {
							  {0x14, 0x02}, 10, {goWalk, goWalk, goSouth, goSouth, goWalk, goWalk, goSouth, goSouth} // waitWest
							 },
							 
               {
							  {0x21, 0x02}, 10, {goSouth, waitSouth, goSouth, waitSouth, waitSouth, waitSouth, waitSouth, waitSouth} // goSouth
							 },
							 
               {
							  {0x22,0x02}, 10, {goWalk, goWest, goWalk, goWest, goWalk, goWest, goWalk, goWalk} // waitSouth
							 },
							 
               {
							  {0x24, 0x08}, 10, {goWalk, waitWalk, waitWalk, waitWalk, goWalk, waitWalk, waitWalk, waitWalk} // goWalk
							 },
							 
               {
							  {0x24, 0x02}, 10,{goWest, goWest, goSouth, goWest, goWest, goWest, goSouth, goWest} // waitWalk
							 }
							 
              };

							 
// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****

							 						 
							 
// Initializing input/sensor ports


void PortB_init(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002;      // 1) B
	delay = SYSCTL_RCGC2_GPIOB; 
	GPIO_PORTB_CR_R = 0x3F;           // allow changes to PF4-0
	GPIO_PORTB_AMSEL_R = 0x00; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R = 0x00000000; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R = 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x00; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R = 0x3F;    // 7) enable digital on PB5-0
}

void PortE_init(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000010;      // 1) E
	
	delay = SYSCTL_RCGC2_GPIOE;      // 2) no need to unlock
	
	GPIO_PORTE_CR_R = 0x07;           // allow changes to PE2-0
  GPIO_PORTE_AMSEL_R = 0x00; // 3) disable analog function on PE1-0
  GPIO_PORTE_PCTL_R = 0x00000000; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R = 0x00;   // 5) inputs on PE1-0
  GPIO_PORTE_AFSEL_R = 0x00; // 6) regular function on PE1-0
  GPIO_PORTE_DEN_R = 0x07;    // 7) enable digital on PE1-0
}



// Initializing output2/pedestrian_signal ports
void PortF_init(void)
{
	 volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0A;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x0A;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x0A;          // 7) enable digital pins PF4-PF0
}



// SysTick Timer

#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
	
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
   SysTick_Wait(800000);  // wait 10ms
  }
}


unsigned long Current_State;
unsigned long Input;
int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	PortB_init();
	PortE_init();
	PortF_init();
	SysTick_Init();
  EnableInterrupts();
	Current_State = goWest;
	
  while(1) // the condition for which loop can run for infinite time
		{
			
		  int WaitWalk = ((Input == 0x01 || Input == 0x02 || Input == 0x03 || 
			                 Input == 0x05 || Input == 0x06 || Input == 0x07) 
		                  && PEDSTRIAN_SIGNAL == 0x08); //for detecting the WaitWalk state.
		
		  // If WaitWalk state detected then toggle the Red signal of the PEDESTRIAN SIGNAL 2 times.
		  if(WaitWalk)
		  {
				
			  int i=0;
			  while(i< 2) // condition for looping through it 2 times only
			  {
					
			    PEDSTRIAN_SIGNAL = 0x02; // the red signal is on
			    SysTick_Wait10ms(10); // delay
			    PEDSTRIAN_SIGNAL = 0x00; // the red signal is off
				  SysTick_Wait10ms(10); // delay
				  i++; // i = i + 1
					
			  }
			  PEDSTRIAN_SIGNAL = 0x02; // the red signal is off
				
		  }
		
		  CAR_SIGNAL = FSM[Current_State].Output[0]; // Set Traffic light for West/South
		
		  PEDSTRIAN_SIGNAL = FSM[Current_State].Output[1]; // Set Traffic light for West/South
		
		  SysTick_Wait10ms(FSM[Current_State].Time); // Waitig time in a valid state/Time delay.
		
		  Input = SENSOR; //SENSORS values means the states of PE2-PE0,possible 
		                 //states are - 000,001,010,011,100,101,110,111 or 0,1,2,3,4,5,6,7
		
		  Current_State = FSM[Current_State].NextState[Input]; // What is the state after receiving 
		                                                      // the Input (depends on the Input and Currnt state)
    }
		
  }

