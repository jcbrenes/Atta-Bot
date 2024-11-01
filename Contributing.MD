# Code Standard for Contributions to Atta-Bot

_Prepared by Juan Carlos Brenes_

This is a formatting and documentation guide for the C++ code in the project. Please follow this guide to ensure the code is readable and understandable by others, helping to make the project more sustainable over time.

# Variables:

- Names should always start with a lowercase letter.

- Use descriptive names, even if they are long.
`contadorPulsosMotorDerecho // **good**`
`sum, j, pos // **bad**`

- If a variable will not be modified in the code, it should be declared as `const`. This helps the compiler make the code more efficient.
`const int velocidadRefDerecha`

- Avoid using magic numbers (constants) directly in the code. While they may make sense at the moment, they can be forgotten after 48 hours. Instead, create a constant variable at the beginning of the code.
`if (contadorPulsosMotorDerecho > 100) // **bad**`
`if (contadorPulsosMotorDerecho > velocidadRefDerecha) // **good**`

# Functions and Classes:

- Names should start with an uppercase letter.

- Use descriptive names.

- Each function should have a description box above the function name, detailing what the function does, its parameters, and its return value. This format allows documentation tools to recognize it.

```cpp
//******************************************************************************************************************
// Function that increments a counter with each encoder pulse. (One summary sentence)
//
// An optional paragraph can provide a more detailed explanation of the function.
//
// @param contadorPulsosMotorDerecho Variable that tracks the encoder pulse count.
// @param incrementoPulso Variable specifying the magnitude of the pulse increment.
//
// @return contadorIncrementado Returns an integer, the general pulse counter plus the encoder increment. (Omit if void)
//******************************************************************************************************************
int IncrementoContador (int contadorPulsosMotorDerecho, int incrementoPulso){
  int contadorIncrementado = contadorPulsosMotorDerecho + incrementoPulso;
  return contadorIncrementado;
}

# Other Considerations:
- With descriptive names, the code should be self-explanatory. However, additional comments can be added as needed.

- Use // for comments rather than /* */, as this facilitates debugging.

- Use spaces to improve code readability. 
```sh
buffer=contador+indice;  //  **cluttered**
buffer = contador + indice;  // **clearer**
```
- Remember to maintain indentation, using only tabs for indentation. Do not use spaces.

- Always use curly braces {} with if, for, while, etc., even if they contain only one line. Omitting them can lead to many errors.
