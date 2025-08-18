# pysimplegui is not able to programmatically switch between tabs
# It just implements tab hide, but once hidden, the tab content disappears forever :-(

import tkinter as tk
from tkinter import ttk

# Function to switch to tab1
def switch_to_tab1():
    tab_control.select(tab1)

# Function to switch to tab2
def switch_to_tab2():
    tab_control.select(tab2)

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

# Run the application
root.mainloop()
