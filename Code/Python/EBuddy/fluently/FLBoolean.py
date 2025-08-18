# Problem: the and operator is not commutative if one of the operands is not boolean (it is None)
cond1 = False
cond2 = None
cond3 = cond1 and cond2 # False and None = False
cond4 = cond2 and cond1 # None and False = None (NoneType)

# We want: None and False = False, and not None as cond1 and cond2 would return (wrong result)
# We want: None and None = None
# We want: True and None = True
# We want: None and True = True
# GPT4 did not make it :-(, here the problem formulated for him:
# Write python code so that the operator "my_and" is implemented correctly
# True my_and True = True # Here my_and == and
# True my_and False = False # Here my_and == and
# False my_and True = False # Here my_and == and
# False my_and False = False # Here my_and == and
# True my_and None = True # None has no weight
# None my_and True = True # None has no weight
# False my_and None = False # None has no weight
# None my_and False = False # None has no weight
# None my_and None = None # None wins only in this case
def bool_eval_and_with_none(cond1, cond2):
    # False and None already returns False as expected
    result = cond1 and cond2
    # None and False returns None, but we want to return None (NoneType): fix this
    # Suboptimal but effective implementation
    if cond1 == True and cond2 == None:
        result = True
        return result
    if cond1 == None and cond2 == True:
        result = True
        return result
    # We trust the check step more than the do step, which may be correct but have a low score, that incorrectly evaluates it as incorrect
    if cond1 == False and cond2 == True:
        result = True
        return result
    if result == None and (cond1 == None and not cond2 == None):
        result = False
        return result
    else:
        return result

def bool_eval_and_with_none_GPT4(a, b):
    if a is None or b is None:
        return None
    elif a:
        return b
    else:
        return a

def bool_eval_and_with_none_test():
    # 3 * 3 = 9 cases
    test_cases = [
        (True, True),
        (True, False),
        (False, True),
        (False, False),
        (True, None),
        (None, True),
        (False, None),
        (None, False),
        (None, None)
    ]

    for cond1, cond2 in test_cases:
        result = bool_eval_and_with_none(cond1, cond2)
        print(f"{cond1} and {cond2} = {result}")

####################################################################
cond1 = True
cond2 = None
cond3 = cond1 and cond2  # True and None = False as expected
cond4 = cond2 and cond1  # None and True = None (NoneType) not expected!!!

cond1 = None
cond2 = None
cond3 = cond1 and cond2 # None and None = None (NoneType)
cond4 = cond2 and cond1 # None and None = None (NoneType)

# Pay attention, if cond1 type is NoneType, False and None = None!
# In this case do following: cond1_new = cond1 or False, to convert it to a boolean type
cond3 = False
cond4 = None
cond5 = cond1 and cond2 # False and None = False

pass

if __name__ == '__main__':
    bool_eval_and_with_none_test()