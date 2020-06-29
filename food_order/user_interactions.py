from enumeration.enum_classes import Items


class user_order():
    def __init__(self):
        pass

    def receive_order_user(self):
        print("-----------------")
        print("Available Menu")
        print("Item # |   Item  ")
        print("-----------------")
        print("1 - Cola\n2 - Pespi\n3 - Coffee")

        done = False

        while done == False:
            order = input("Enter item number to order : ")

            try:
                int_order = int(order)
                if int_order > 0 and int_order <= len(Items):
                    print("Entered order : ", Items(int_order).name)
                    return int_order
                else:
                    print("Invalid item number")
            except AssertionError as e:
                raise (AssertionError("You have'nt entered valid an integer. Enter a valid item number"))
