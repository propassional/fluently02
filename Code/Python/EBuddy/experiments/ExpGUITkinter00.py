import tkinter as tk
import psutil

def hello_world():
    root = tk.Tk()
    root.title("EBuddy Warning")
    # Create a label widget
    label = tk.Label(root, text="Hello World")
    label.pack() # label.pack(pady=10)
    # Create a button widget
    # button = tk.Button(root, text="Say Hello", command=say_hello)
    # button.pack()
    # Quit the window with the "Esc" button: bind the escape key to the quit_window function
    root.configure(bg='red')
    root.mainloop()  # Start the main event loop

def window_quit_button(event):
    # global is not needed since root here is only read
    root.destroy()

def window_quit(): # No event here
    # global is not needed since root here is only read
    root.destroy()

def window_close_after_some_seconds(text_input, auto_quit=True):
    basis_message = f'E-Buddy message '
    if auto_quit:
        basis_message += f'(disappears in {window_close_interval_in_sec} seconds)\n'
    message = basis_message + text_input
    if not auto_quit:
        message += "\n(Quit with Esc)"

    global root # Needed for window_quit
    root = tk.Tk()
    #text1 = tk.Text(root, height=3, font=("Arial", 20))  # font=("Arial", 14, "bold"))
    #text1 = tk.Text(root, height="auto", font=("Arial", 20))  # font=("Arial", 14, "bold"))


    label = tk.Label(root, text=message, font=("Arial", 20))
    label.pack()

    #text1.insert(tk.END, message)
#    text1.pack()

    root.title("EBuddy Warning")
    root.configure(bg='red')
    root.bind("<Escape>", window_quit_button)  # Bind the escape key to the quit_window function
    # Set window attributes to bring it to the foreground
    root.lift()
    root.attributes("-topmost", True)
    root.attributes("-topmost", False)
    # Window auto-quits after some seconds
    if auto_quit:
        root.after(window_close_interval_in_ms, window_quit)

    label_width = label.winfo_reqwidth()
    label_height = label.winfo_reqheight()
    window_width = int(1.2 * label_width)
    window_height = int(1.2 * label_height)

    # Set the window size
    root.geometry(f"{window_width}x{window_height}")

    root.mainloop()  # Start the main event loop

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