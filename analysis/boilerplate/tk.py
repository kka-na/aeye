import tkinter as tk
import tkinter.ttk as ttk

if __name__ == '__main__':
    window = tk.Tk()
    window.rowconfigure([0], weight=1, minsize=100)
    window.columnconfigure([0,1], weight=1, minsize=100)

    s = ttk.Style()
    s.configure('out.TFrame', background="white")

    button_frame = ttk.Frame(master=window, width=100, height=100)
    output_frame = ttk.Frame(master=window, width=500, height=100, style='out.TFrame')

    button_frame.grid(row=0, column=0)
    output_frame.grid(row=0, column=1)

    button_store = ttk.Button(master=button_frame, text="Store Current Signals")
    button_compare = ttk.Button(master=button_frame,text="Compare Signals")
    output = tk.Text(output_frame)

    scollb = ttk.Scrollbar(command=output.yview)
    scollb.grid(row=0, column=2, sticky='nsew')
    output['yscrollcommand'] = scollb.set

    string = 'aaaaaaaaaaaaaaaaaaaaa'

    output.insert(tk.END, string)

    output.pack()
    button_store.pack()
    button_compare.pack(padx=5, pady=10)

    window.mainloop()


