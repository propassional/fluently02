# All classes inherit from BaseClass, but only Class1 has to explicitly inherit from BaseClass,
# since all other classes inherit from BaseClass through Class1

class BaseClass:
    def __init__(self, name):
        self.name = name

class Class1(BaseClass):
    def __init__(self, name, salary):
        super().__init__(name)
        self.salary = salary

class Class2(Class1):
    def __init__(self, name, salary, title):
        super().__init__(name, salary)
        self.title = title

class Class3(Class2):
    def __init__(self, name, salary, title, lesson):
        super().__init__(name, salary, title)
        self.lesson = lesson

# Create an instance of Class3
obj = Class3(name="MyObject", salary=1000, title="Professor", lesson="Mathematics")

# Print the attributes
print(f"Name of the object: {obj.name}")
print(f"Salary of the object: {obj.salary}")
print(f"Title of the object: {obj.title}")
print(f"Lesson of the object: {obj.lesson}")
