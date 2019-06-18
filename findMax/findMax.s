#define s1 $17 // store address where array begins
#define s2 $18 // store array size, it is in data memory at 0x08

#define a0 $4  // function's parameter of address array
#define a1 $5  // function's parameter of size array (number of elements)
#define t0 $8  // function's val, index of array
#define t1 $9  // function's val, array value at index t0
#define t2 $10 // function's val, max value in array
#define t3 $11 // function's val, boolean val for comparing
#define v0 $2  // function's return val

.globl    array       // label "array" is declared as global. It is visible from all files in the project.
.data                 // directive indicating start of the data segment
.align    2           // set data alignment to 4 bytes

array:                 // label - name of the memory block
.word   4, 7, 11, 3, 5    // values in the array to increment...

.text                 // beginning of the text segment (or code segment)
.globl start
.ent start


start:
  addi s1, s1, 0xC   /* addi s1, s1, 0xC */   /* la s1, array */ 
  lw s2, 0x8($0) /* lw s2, 0x8($0) */     /* addi s2, $0, 5 */ 

  add a0, $0, s1  // fction param array pointer
  add a1, $0, s2  // fction param size array
  jal max
  sw v0, 0x4($0)
  
//------------------------------------------------

max:
  addi t0, $0, 0x0
  lw t2, 0x0(a0)
  beq a1, $0, done2
  for:
    beq a1, t0, done1
    lw t1, 0x0(a0)
    //
    slt t3, t2, t1 
    beq t3, $0, noNewMax
    addi t2, t1, 0x0
    noNewMax:
    //
    addi t0, t0, 0x1
    addi a0, a0, 0x4
    jal for
  done1:
    addi v0, $0, 0x0
    jr $31
  done2:
    addi v0, t2, 0x0
    jr $31

//------------------------------------------------ 

nop
.end start

