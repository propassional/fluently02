import tkinter as tk
import psutil

def quit_window(event):
    # global is not needed since root here is only read
    root.destroy()

# def say_hello():
#     global label # Needed since label is not only read, but also modified
#     label.config(text="Hello, World!")

def main():
    global root
    root = tk.Tk()
    root.title("Warning")

    # Create a label widget
    label = tk.Label(root,text="Hello World")
    label.pack(pady=10)

    # Create a button widget
    # button = tk.Button(root, text="Say Hello", command=say_hello)
    # button.pack()

    # Bind the escape key to the quit_window function
    root.bind("<Escape>", quit_window)

    # Start the main event loop
    root.mainloop()
    
def method():
    global label, root
    process_name = "astudio_pro.exe"

    for proc in psutil.process_iter():
        if proc.name() == process_name:
            # Create the main window
            root = tk.Tk()
            root.title("Warning")

            # Create a label widget
            label = tk.Label(root,
                             text="Please save now your work in Artec Studio, since it will be killed after you close this window (Esc)")
            label.pack(pady=10)

            # Create a button widget
            # button = tk.Button(root, text="Say Hello", command=say_hello)
            # button.pack()

            # Bind the escape key to the quit_window function
            root.bind("<Escape>", quit_window)

            # Start the main event loop
            root.mainloop()

    return True

if __name__ == '__main__':
    main()