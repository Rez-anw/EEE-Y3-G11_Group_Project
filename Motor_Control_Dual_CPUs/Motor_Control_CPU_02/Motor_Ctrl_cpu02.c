// FILE: Lab9_cpu02.c

#include "F28x_Project.h"

// Place and read the array in specific memory section that is shared by both CPUs
#pragma DATA_SECTION(shared_array, "Shared_RAM") // Place in shared memory
volatile uint32_t shared_array[20];                // Example array

// Variables
int i = 0;
uint32_t received_array[20];

// Function Prototypes
interrupt void ipc2_isr(void);

void main(void)
{
    // Initialize System Control for CPU2
    InitSysCtrl();

    // Clear all interrupts and initialize PIE control registers
    DINT;
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table
    InitPieVectTable();

    // Map ISR functions
    EALLOW;
    PieVectTable.IPC1_INT = &ipc2_isr; // Map IPC1 interrupt to ISR function
    EDIS;

    // Clear IPC flags
    IpcRegs.IPCCLR.bit.IPC0 = 1;        // Clear IPC0 flag to prevent interference
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE group 1

    // Enable PIE interrupts
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1; // Enable IPC1 interrupt in PIE group 1
    PieCtrlRegs.PIEIFR1.bit.INTx13 = 1; // Force initial IPC0 interrupt

    // Enable global interrupts and real-time interrupt events
    IER |= M_INT1; // Enable CPU interrupt group 1
    EINT;          // Enable global interrupts (INTM)
    ERTM;          // Enable real-time debug interrupt (DBGM)

    // Notify CPU1 that CPU2 is ready
    IpcRegs.IPCSET.bit.IPC17 = 1; // Set IPC17 to signal readiness to CPU1

    // Main loop
    while(1)
    {
        ; // Idle loop, waiting for interrupts
    }
}

// IPC interrupt service routine for CPU2
interrupt void ipc2_isr(void){
    // Read the array in shared memory and and save it in a new array
    for (i = 0; i < 20; i++) {
        received_array[i] = shared_array[i];
    }


    // Return from interrupt
    IpcRegs.IPCACK.bit.IPC1 = 1;           // Clear IPC1 flag to acknowledge CPU1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE group 1 to enable further interrupts
}

// end of file
