# The issue is that environment variables set in one script (moduleA.py)
# are not automatically available to another script (moduleB.py) when modules run separately.
# Each script runs in its own process, and environment variables set in one process do not persist to another.
# To fix this, you can set the environment variable in the shell before running the second script,
# or you can use a configuration file or another method to share the SM_JSON_data between the scripts.

# moduleA.py
import os
os.environ['MY_STRING'] = 'Hello from moduleA'

# moduleB.py
import os
my_string = os.getenv('MY_STRING')
print(my_string)  # Output: Hello from moduleA


