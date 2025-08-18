# pysimplegui is not able to programmatically switch between tabs
# It just implements tab hide, but once hidden, the tab content disappears forever :-(
# This example implements a switch between two tabs, that are updated on a regular time base
import time
import tkinter as tk
from tkinter import ttk

# Function to switch to tab1
def switch_to_tab1():
    tab_control.select(tab1)
    print("tab_control.select(tab1)")
    time.sleep(0.5)

# Function to switch to tab2
def switch_to_tab2():
    tab_control.select(tab2)
    print("tab_control.select(tab2)")
    time.sleep(0.5)

def update_tab1_content():
    switch_to_tab1()
    # Read the current text
    old_text = text1.get("0.0", tk.END)
    old_text = old_text.replace("\n", "")
    # Clear the existing content
    text1.delete(1.0, tk.END)
    # Insert the new text
    new_text1 = "Guantanamera"
    new_text2 = "Guajiro Guantanamera"
    if old_text != new_text1:
        text1.insert(tk.END, new_text1)
    else:
        text1.insert(tk.END, new_text2)
    # Schedule the next update after 1000 milliseconds (1 second)
    root.after(1000, update_tab1_content)

def update_tab2_content():
    switch_to_tab2()
    text2.insert(tk.END, "Hasta siempre...")
    root.after(2000, update_tab2_content)

def window_quit_button(event):
    # global is not needed since root here is only read
    root.destroy()

def window_quit(): # No event here
    # global is not needed since root here is only read
    root.destroy()

def window_close_after_some_seconds(text_input, auto_quit=True):
    window_close_interval_in_ms = 5000
    window_close_interval_in_sec = window_close_interval_in_ms / 1000

    basis_message = f'E-Buddy message '
    if auto_quit:
        basis_message += f'(disappears in {window_close_interval_in_sec} seconds)\n'
    message = basis_message + text_input
    if not auto_quit:
        message += "\n(Quit with Esc)"

    global root # Needed for window_quit
    root = tk.Tk()
    label = tk.Label(root, text=message, font=("Arial", 20))
    label.pack()
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
    # Start the main event loop
    root.mainloop()

if __name__ == '__main__':
    window_close_after_some_seconds("Test message", False)

    # Create the main window
    root = tk.Tk()
    root.title("Tab Switcher")

    # Create the Tab Control
    tab_control = ttk.Notebook(root)

    # Create the layout for the first tab with a Text widget
    tab1 = ttk.Frame(tab_control)
    tab_control.add(tab1, text='Tab 1')
    text1 = tk.Text(tab1, height=10, width=40)
    text1.pack(padx=5, pady=5)

    # Create the layout for the second tab with a Text widget
    tab2 = ttk.Frame(tab_control)
    tab_control.add(tab2, text='Tab 2')
    text2 = tk.Text(tab2, height=10, width=40)
    text2.pack(padx=5, pady=5)

    # Add the tab control to the main window
    tab_control.pack(expand=1, fill="both")

    # Create buttons to switch tabs
    btn_switch_to_tab1 = ttk.Button(root, text="Go to Tab 1", command=switch_to_tab1)
    btn_switch_to_tab1.pack(side=tk.LEFT, padx=(20, 10), pady=(10, 10))

    btn_switch_to_tab2 = ttk.Button(root, text="Go to Tab 2", command=switch_to_tab2)
    btn_switch_to_tab2.pack(side=tk.RIGHT, padx=(10, 20), pady=(10, 10))

    update_tab1_content()
    update_tab2_content()

    # Run the application
    root.mainloop()
