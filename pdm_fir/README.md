# PDM bitstream FIR filter

The code utilizes 2 simple approaches to minimize computational efforts:
- circular buffer
- work with words rather than individual bits of the PDM stream

The latter requires using more memory for coefficients but flush is pretty
cheep now. The provided python script is used to automate coefficients generation.
