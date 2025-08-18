class ClassA:
    def __init__(self, name):
        self.name = name

class ClassB(ClassA):
    def __init__(self, name, age):
        super().__init__(name)
        self.age = age

# Create an instance of ClassB
objB = ClassB(name="John", age=30)

# Access the "name" attribute of objB
print(f"objB's name: {objB.name}")

# Create an instance of ClassA
objA = ClassA(name="Alice")

# Access the "name" attribute of objA
print(f"objA's name: {objA.name}")

if __name__ == '__main__':
    # The interesting part above is that objB can access "name", even if it is defined within the super class
    pass