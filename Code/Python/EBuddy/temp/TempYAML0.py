import yaml

def join(loader, node):
    seq = loader.construct_sequence(node)
    return ''.join([str(i) for i in seq])

yaml.add_constructor('!join', join)

yaml_data = """
user_dir: &DIR /home/user
user_pics: !join [*DIR, /pics]
"""

result = yaml.load(yaml_data)
print(result)