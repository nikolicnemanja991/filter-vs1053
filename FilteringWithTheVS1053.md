# Introduction #

The goal of this project was to implement a fast fourier transform on the VS1053 digital signal processor.  After implementing this, the goal then changed to pitch shifting the sounds, which was more challenging than expected. The goal was changed to filtering specific frequencies, which gave me a better understanding of the FFT algorithm.


# Coding Techniques #

The VS1053 is programmed using C. The DSP is still picky about how you write the code in order to run it's processes in parallel. It is best to:
  * use s\_int16 and u\_in16 data types instead of the short data type
  * use pointers to alter values in a buffer, rather than index notation (i.e (asterisk)buffer rather than buffer(i))
  * do not reuse buffers
  * do not alter data in the input and output buffers

# Wiring #

Usually with the VS1053 Simple DSP, you have to wire the processor a specific way to allow correct functionality. Since I used the VS1053 Simple Host Board, I did not have to do this wiring. For correct wiring of the VS1053 without a host board, refer to the data sheet here: http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf.

# How It Works #
Once I set up the DSP correctly, as well as learned the proper coding techniques, an FFT method needed to be implemented in C. Instead of writing my own, I edited a method written by Thomas Roberts, and later enhanced by other developers: https://code.google.com/p/matching-logic/source/browse/trunk/ml-k/C/programs/fix-fft.c?spec=svn230&r=197. Once the method was implemented, all that was left to do was to use it for my specific application. The input from the line in is fetched and put into a buffer. The data is then fed into another buffer, reordering the data to have all even indexed samples at the beginning and all odd indexed samples to put at the end. Once the data is reordered, it is ready to be transformed. After the transform occurs, certain frequencies can be taken out by setting their index's value to zero. The inverse fourier transform will return a somewhat flawed version of the final output and a scale value. As the data is reordered to be sequential (the data still has even samples at the beginning and odd numbers at the end), the bits in a sample are shifted left by the scale value, generating the true final output. This final output is fed to the output buffer.

# Demo #
<a href='http://www.youtube.com/watch?feature=player_embedded&v=BRnhQPnAuEc' target='_blank'><img src='http://img.youtube.com/vi/BRnhQPnAuEc/0.jpg' width='425' height=344 /></a>

# Future Goals #
The next goal I have set myself is to develop a pitch shifting algorithm for a guitar pedal using my new-found knowledge of the fast fourier transform and digital signal processing. This most likely will require either an extra algorithm to calculate true frequencies of a signal (as the FFT returns only approximations that increase in accuracy with larger amounts of samples) or developing a FFT that only looks for guitar frequencies. In the future I want to use a different processor, as the VS1053 does not have many options for digital or analog input, which would be useful for changing specific parameters of an effect.