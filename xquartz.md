# xQuartz

I stumbled across this thing called xQuartz while doing this project. I wanted to see the live graph on my computer from the raspberry pi that is getting UART data in MAVLink packet.

It has quite some amount of datas, and I wanted to see if the parser I wrote makes sense with what is going on with the actual hardware. Yes, I could just stare at the numbers that are sent tens of times every seconds, but instead, I thought it would be cool to do it this way for reasons being: hell, I like graph more than text and it would be way more convenient for me to debug and would make the final look a lot nicer.

I decided to use xQuartz because it was first thing that I was able to find that can do something I wanted. xQuartz is basically a **Display Server**. It is an essential part of the GUI (graphical user interrface). It manages graphical display output and handles user inputs. It is an intermediary between the appplication and the harware that displays something on the screen.

The protocol that is used is X Window System (X11 is version 11 for it, and xQuartz is X11 implementation for macOS). Roughly the process looks like as such: Linux/Unix GUI app -> X11 Protocol -> xQuartz -> macOS GUI.
