# launch_xml

This package provides an abstraction of the XML tree.

## XML front-end mapping rules

### Accessing xml attributes

When having an xml tag like:

```xml
<tag value="2"/>
```

If the entity `e` is wrapping it, the following statements will be true:
```python
e.get_attr('value') == '2'
e.get_attr('value', data_type=int) == 2
e.get_attr('value', data_type=float) == 2.0
```

By default, the value of the attribute is returned as a string.

Allowed types are:
    - scalar types: `str, int, float, bool`
    - An uniform list, e.g.: `List[int]`.
    - The list of entities type: `List[Entity]` (see below).

`List` is the usual object from the `typing` package.
`data_type` can also be set to `None`, in which case yaml rules will be used.

For handling lists, the `*-sep` attribute is used. e.g.:

```xml
<tag value="2,3,4" value-sep=","/>
<tag2 value="2 3 4" value-sep=" "/>
<tag3 value="2, 3, 4" value-sep=", "/>
```

```python
tag.get_attr('value', data_type=List[int]) == [2, 3, 4]
tag2.get_attr('value', data_type=List[float]) == [2.0, 3.0, 4.0]
tag3.get_attr('value', data_type=List[str]) == ['2', '3', '4']
```

In the case a value can be either an instance of a type or a substitution, the `can_be_str` argument of `get_attr` must be used, followed by a call to `parser.parse_if_substitutions`:

```python
value = e.get_attr('value2', data_type=int, can_be_str=True)
normalized_value = parser.parse_if_substitutions(value)
```

For checking if an attribute exists, use an optional argument:

```python
value = e.get_attr('value', optional=True)
if value is not None:
    do_something(value)
```

With `optional=False` (default), `AttributeError` is raised if the specified attribute is not found.

### Accessing XML children as attributes:

In this xml:

```xml
<executable cmd="ls">
    <env name="a" value="100"/>
    <env name="b" value="stuff"/>
</node>
```

The `env` children could be accessed like:

```python
env = e.get_attr('env', data_type=List[Entity])
len(env) == 2
env[0].get_attr('name') == 'a'
env[0].get_attr('value') == '100'
env[1].get_attr('name') == 'b'
env[1].get_attr('value') == 'stuff'
```

In these cases, `e.env` is a list of entities, that can be accessed in the same abstract way.

### Accessing all the XML children:

All the children can be directly accessed:

```python
e.children
```

It returns a list of `launch_xml.Entity` wrapping each of the XML children.

## Built-in substitutions

See [this](https://github.com/ros2/design/blob/d3a35d7ea201721892993e85e28a5a223cdaa001/articles/151_roslaunch_xml.md) document.
