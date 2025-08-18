# When you run this code, it will print “This is a sample method.”
# This demonstrates that the sample_method has been successfully added to the foo instance of the Foo class.

import types

# This method does not belong to the class Foo
def sample_method(self):
    print('This is a sample method.')

class Foo:
    pass

if __name__ == '__main__':
    foo = Foo()

    # Method is dynamic binded
    foo.sample_method = types.MethodType(sample_method, foo)

    # This will print “This is a sample method.” This demonstrates that the sample_method has been successfully added to the foo instance of the Foo class.
    foo.sample_method()