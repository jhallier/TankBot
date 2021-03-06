import serial
import time
import pygame

ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=5)
time.sleep(3) # wait for Arduino to establish communication  

pygame.init()

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
 
# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Joystick input")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop

    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()
    
    # For each joystick:
    # If you have more than one joystick, change the number here
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
        
    speed = - joystick.get_axis(1) # needs to be negated
    steering = joystick.get_axis(0)
    textPrint.print(screen, "Speed value: {:>2.2f}".format(speed))
    textPrint.print(screen, "Steering value: {:>2.2f}".format(steering))
    textPrint.unindent()

    ''' Serial msg in the form XXXYYY
    XXX is velocity with 0 full backward, 127 stop, 254 full ahead
    YYY is steering with 0 leftmost, 127 middle, 254 rightmost
    '''
    serial_msg = ""
    if abs(speed) < 0.05:
        speed_conv = 127
    else:
	    speed_conv = int(speed * 127 + 127)
    steer_conv = int(steering * 127 + 127)

    # Padding to make sure the string is three characters long, 7 -> 007
    pad_speed, pad_steer = "", ""
    if speed_conv < 10:
        pad_speed = "00"
    elif speed_conv < 100: 
        pad_speed = "0"
    if steer_conv < 10:
        pad_steer = "00"
    elif steer_conv < 100:
        pad_steer = "0"
    serial_msg = pad_speed + str(speed_conv) + pad_steer + str(steer_conv) + "\n"
    textPrint.print(screen, "Serial message: {0}".format(serial_msg))
    ser.write(serial_msg.encode())
    ser.flush() #

    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 10 frames per second
    clock.tick(10)
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()

