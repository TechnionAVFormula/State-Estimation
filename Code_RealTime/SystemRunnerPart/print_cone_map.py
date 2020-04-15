# Get proper Enum:
YELLOW = messages.perception.Yellow
BLUE = messages.perception.Blue
ORANGE = messages.perception.Orange



def print_map(cone_array):
    root = Tk()
    my_canvas = Canvas(root, width=1200, height=600)   
    
    for cone in cone_array:
        if cone.type == BLUE:
            my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="blue")
        if cone.type == YELLOW:
            my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="yellow")
        
    my_canvas.pack()
    root.mainloop()