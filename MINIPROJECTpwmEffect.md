# Explanation #

This project simply takes an input and adds an effect to it before outputting it. The processor first takes the input, feeds it to a buffer. The buffer then throws out samples below a certain amplitude, while assigning samples above that amplitude to a fixed amplitude. It then throws out samples that are within a specific indexes. This essentially turns the signal into a square pulse whose width can be altered by pushing a button on the VS1053. This altered signal is then outputted.

# Demo #

<a href='http://www.youtube.com/watch?feature=player_embedded&v=ufxJP3LU3B0' target='_blank'><img src='http://img.youtube.com/vi/ufxJP3LU3B0/0.jpg' width='425' height=344 /></a>