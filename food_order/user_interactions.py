from enumeration.enum_classes import Items


class user_order():
    def __init__(self):
        pass

    def receive_order_user(self):
        print("-----------------")
        print("Available Menu")
        print("Item # |   Item  ")
        print("-----------------")
        print("1 - Coffee\n2 - Sprite\n3 - Cola")

        done = False
        order = []

        while done == False:
            order = input("Enter item number to order : ")

            int_order = int(order)

            if int_order > 0 and int_order <= len(Items):
                print("Entered order : ", Items(int_order).name)
                order.append(int_order)
                another = input("Do you want to order more: y/n")
                if another == 'n':
                    done

            else:
                print("Invalid item number")

        return int_order