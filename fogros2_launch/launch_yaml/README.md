# launch_yaml

This package provides an abstraction of the YAML tree.

## YAML front-end mapping rules

### Accessing yaml attributes

When having an YAML file like:

```yaml
tag:
    value1: '2'
    value2: 2
    value3: 2.0
```

If the entity `e` is wrapping `tag`, the following statement will be true:
```python
e.get_attr('value1') == '2'
e.get_attr('value2', data_type=int) == 2
e.get_attr('value3', data_type=float) == 2.0
```

By default, `get_attr` returns an string and it does type checking. The following code will raise a `TypeError`:

```python
e.get_attr('value1', data_type=int)
e.get_attr('value2', data_type=float)
e.get_attr('value3')
```

Allowed types are:
    - scalar types: `str, int, float, bool`
    - An uniform list, e.g.: `List[int]`.
    - The list of entities type: `List[Entity]` (see below).

`List` is the usual object from the `typing` package.
`data_type` can also be set to `None`, in which case any of the following scalar types or uniform lists of them are allowed:

```python
int, float, bool, str
```

In the case a value can be either an instance of a type or a substitution, the `can_be_str` argument of `get_attr` must be used, followed by a call to `parser.parse_if_substitutions`:

```python
value = e.get_attr('value2', data_type=int, can_be_str=True)
normalized_value = parser.parse_if_substitutions(value)
```

For checking if an attribute exists, use optional argument:

```python
value = e.get_attr('value', optional=True)
if value is not None:
    do_something(value)
```

With `optional=False` (default), `AttributeError` is raised if it is not found.

### Accessing attributes that are also an Entity:

In this yaml:

```yaml
executable:
    cmd: ls
    env:
        - name: a
        - value: '100'
        - name: b
        - value: 'stuff'
```

The `env` children could be accessed doing:

```python
env = e.get_attr('env', data_type=List[Entity])
len(env) == 2
env[0].get_attr('name') == 'a'
env[0].get_attr('value') == '100'
env[1].get_attr('name') == 'b'
env[1].get_attr('value') == 'stuff'
```

In these cases, `e.env` is a list of entities, that can be accessed in the same abstract way.

### Accessing children:

All the children can be directly accessed. e.g.:

```yaml
group:
    - executable:
        cmd: ls
    - executable:
        cmd: ps
```

```python
e.children
```

or:

```yaml
group:
    scoped: False
    children:
        - executable:
            cmd: ls
        - executable:
            cmd: ps
```

```python
e.children
```

It returns a list of launch_xml.Entity wrapping each of the xml children.
In the example, the list has two `Entity` objects wrapping each of the `executable` tags.

## Built-in substitutions

See [this](https://github.com/ros2/design/blob/d3a35d7ea201721892993e85e28a5a223cdaa001/articles/151_roslaunch_xml.md) document.
