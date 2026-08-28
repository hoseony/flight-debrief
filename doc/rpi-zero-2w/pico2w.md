_This dir was made to test uart on the rpi zero 2w._

# Setting Up Raspberry Pi Pico Zero 2w
rpi zero 2w: OS Lite (64bit)

## Testing UART
_./uart_loopback_test.c_

This turned out to be a little more complicated than I expected. Unlike other bare-metal projects that I did, because the program runs on Linux, it needs to interact with the hardware through Unix system calls and device files. A lot more learnings to do, but this is what I figured so far. Please let me know if there's anything wrong...

`/dev/serial0` is a symbolic links that points to something like /dev/ttyS0. For example, you can check this on terminal:
```
> ls -l /dev/serial0
lrwxrwxrwx 1 root root 5 Aug  4 02:40 /dev/serial0 -> ttyS0
```
The name /dev/ttyS0 can be understood as:
- /dev/: device files 
- tty: terminal/serial interface
- S: UART serial port 
- 0: first port

When the c program calls `open("/dev/serial0", ...)` it, what it really doing is asking Linux Kernel to find the device -> check permissions -> call UART driver -> creates and gives you "file descriptor".

### File Descriptor
When you opens a file, you are open a resource and receive a file descriptor. You give the path name and the kernel gives you back a file descriptor. File descriptor is just an integer that is unique for the process. It is a process local handle to a kernel object. 

File descriptor is an index into the file descriptor table. For each process, the operating system wil have PCB (Process Control Block), which tracks the context of the process. One of the fields is an array called the file descriptor table. It internally keeps track of all the resources that the process owns and can operate on.

This file descriptor table holds pointers to resources. For exmaple, 0: stdin, 1: stdout, 2: stderr. The open() choose the lowest unused descriptor and returns you the file descriptor, and close() removes the entry. 
