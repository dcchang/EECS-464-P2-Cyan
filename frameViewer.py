
import pygame as G
from os import stat, system
from time import sleep, time
from sys import stdout

stdout.write("""
##
## This program will follow and display the frame sequence
##   f0001.png f0002.png ... in the current directory
## Works as at up to 30fps, and updates live
##
## Note: you can convert the frame sequence to a video with the mencoder tool, e.g.
## mencoder "mf://f*.png" -mf fps=15 -o video.avi -ovc lavc -lavcopts vcodec=mpeg4
## You can install mencoder with:
## sudo apt install mencoder
##
## Quit the current program with Ctrl-C
##
""")
scr = G.display.set_mode([1200,800])
G.display.flip()
img = None
fn = None
for n in range(1,10000):
    lfn = fn # Update last filename
    fn = "f%04d.png" % n
    stdout.write("\r"+fn)
    stdout.flush()
    # Wait until the file appears
    while True:
        try:
            if stat(fn):
                break
        except OSError:
            sleep(0.03)
    if lfn: # If there is a "last filename" --> display the image
        img = G.image.load(lfn)
        scr.blit(img,(0,0))
        G.display.update()
