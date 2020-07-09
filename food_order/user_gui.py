
import tkinter as tk
from tkinter import Tk, Label, Button, StringVar, Entry, Text
from enumeration.enum_classes import Items

class GuiInput:
    def __init__(self, master):
        self.master = master
        self.master.title("Order")

        pad_x = 10
        pad_y = 10

        self.user_order = False

        self.orders = [None, [], []]

        self.items = ["Coffee", "Sprite", "Cola"]

        self.table = 0
        self.table_order_state = [False, False, False]

        self.table_1 = Button(self.master, text="Table 1", command=self.choose_table_1, width=30, height=2)
        self.table_1.grid(row=0, column=0, padx=pad_x, pady=pad_y)

        self.table_2 = Button(self.master, text="Table 2", command=self.choose_table_2, width=30, height=2)
        self.table_2.grid(row=1, column=0, padx=pad_x, pady=pad_y)

        self.coffee = Button(self.master, text="Coffee", command=self.choose_coffee, width=30, height=2,
                             state=tk.DISABLED)
        self.coffee.grid(row=2, column=0, padx=pad_x, pady=pad_y)

        self.sprite = Button(self.master, text="Sprite", command=self.choose_sprite, width=30, height=2,
                             state=tk.DISABLED)
        self.sprite.grid(row=4, column=0, padx=pad_x, pady=pad_y)

        self.cola = Button(self.master, text="Cola", command=self.choose_cola, width=30, height=2, state=tk.DISABLED)
        self.cola.grid(row=5, column=0, padx=pad_x, pady=pad_y)

        self.order_button = Button(self.master, text="Order", command=self.order_done, width=30, height=2,
                                   state=tk.DISABLED)
        self.order_button.grid(row=7, column=0, padx=pad_x, pady=pad_y)

        self.confirm_button = Button(self.master, text="Confirm", command=self.order_confirm, width=30, height=2,
                                     state=tk.DISABLED)
        self.confirm_button.grid(row=8, column=0, padx=pad_x, pady=pad_y)

        self.quit = Button(self.master, text="Quit", command=self.quit, width=30, height=2)
        self.quit.grid(row=9, column=0, padx=pad_x, pady=pad_y)

        self.display = Text(self.master, background="black", foreground="white", width=100, height=30)
        self.display.grid(row=0, column=5, rowspan=10, columnspan=3, padx=pad_x, pady=pad_y)
        self.display.insert(tk.END, "Order :")

    def choose_table_1(self):
        self.table = 1
        self.add_message("\n\nPlease enter order for Table 1")
        self.orders[self.table] = []

        self.table_1['state'] = tk.DISABLED
        self.table_2['state'] = tk.NORMAL

        self.coffee['state'] = tk.NORMAL
        self.sprite['state'] = tk.NORMAL
        #self.cola['state'] = tk.NORMAL

    def choose_table_2(self):
        self.table = 2
        self.add_message("\n\nPlease enter order for Table 2")
        self.orders[self.table] = []

        self.table_2['state'] = tk.DISABLED
        self.table_1['state'] = tk.NORMAL

        self.coffee['state'] = tk.NORMAL
        self.sprite['state'] = tk.NORMAL
        #self.cola['state'] = tk.NORMAL

    def add_message(self, message):
        self.display.insert(tk.END, message)

    def choose_coffee(self):
        self.orders[self.table].append(Items.Coffee)
        self.coffee['state'] = tk.DISABLED
        self.order_button['state'] = tk.NORMAL

    def choose_sprite(self):
        self.orders[self.table].append(Items.Sprite)
        self.sprite['state'] = tk.DISABLED
        self.order_button['state'] = tk.NORMAL

    def choose_cola(self):
        self.orders[self.table].append(Items.Cola)
        self.cola['state'] = tk.DISABLED
        self.order_button['state'] = tk.NORMAL

    def show_order(self, table_num):
        for item in self.orders[table_num]:
            self.add_message("\n - " + item.name)

    def order_done(self):
        if len(self.orders[self.table]):
            msg = str("\n\nOrdered Item for table " + str(self.table) + " : \n")
            self.add_message(msg)

            self.show_order(self.table)

            self.table_order_state[self.table] = True
            self.confirm_button['state'] = tk.NORMAL

    def order_confirm(self):
        self.add_message("\n===========================================")
        self.add_message("\nFinal Order Summary : ")
        if self.table_order_state[1]:
            self.add_message("\nTable 1 orders:")
            self.show_order(1)
            self.add_message("\n===========================================")

        if self.table_order_state[2]:
            self.add_message("\nTable 2 orders:")
            self.show_order(2)
            self.add_message("\n===========================================")

        self.add_message("\n\nThanks for your order. You will get your order status in a while")

        self.table_1['state'] = tk.DISABLED
        self.table_2['state'] = tk.DISABLED

        self.coffee['state'] = tk.DISABLED
        self.sprite['state'] = tk.DISABLED
        self.cola['state'] = tk.DISABLED

        self.order_button['state'] = tk.DISABLED
        self.confirm_button['state'] = tk.DISABLED

        self.user_order = True

    def quit(self):
        self.master.quit()

    def get_order(self):
        return self.orders[1], self.orders[2]

    def get_order_stat(self):
        return self.user_order

"""
if __name__ == "__main__":
    root = Tk()
    gui = GuiInput(root)
    root.mainloop()
"""