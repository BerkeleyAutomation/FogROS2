# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Module for the Node action."""

import os
import pathlib
from tempfile import NamedTemporaryFile
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from typing import Union

try:
    import importlib.metadata as importlib_metadata
except ModuleNotFoundError:
    import importlib_metadata

from launch.action import Action
from launch.actions import ExecuteProcess
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.frontend.type_utils import get_data_type_from_identifier

from launch.launch_context import LaunchContext
import launch.logging
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LocalSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.utilities import add_node_name
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import get_node_name_count
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import normalize_parameters
from launch_ros.utilities import normalize_remap_rules
from launch_ros.utilities import plugin_support
from launch_ros.utilities import prefix_namespace


from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

import yaml

from ..descriptions import Parameter
from ..descriptions import ParameterFile


class NodeActionExtension:
    """
    The extension point for launch_ros node action extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods may be defined:
    * `command_extension`
    * `execute`
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(NodeActionExtension, self).__init__()
        plugin_support.satisfies_version(self.EXTENSION_POINT_VERSION, '^0.1')

    def prepare_for_execute(self, context, ros_specific_arguments, node_action):
        """
        Perform any actions prior to the node's process being launched.

        `context` is the context within which the launch is taking place,
        containing amongst other things the command line arguments provided by
        the user.

        `ros_specific_arguments` is a dictionary of command line arguments that
        will be passed to the executable and are specific to ROS.

        `node_action` is the Node action instance that is calling the
        extension.

        This method must return a tuple of command line additions as a list of
        launch.substitutions.TextSubstitution objects, and
        `ros_specific_arguments` with any modifications made to it. If no
        modifications are made, it should return
        `[], ros_specific_arguments`.
        """
        return [], ros_specific_arguments


@expose_action('node')
class Node(ExecuteProcess):
    """Action that executes a ROS node."""

    UNSPECIFIED_NODE_NAME = '<node_name_unspecified>'
    UNSPECIFIED_NODE_NAMESPACE = '<node_namespace_unspecified>'

    def __init__(
        self, *,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        name: Optional[SomeSubstitutionsType] = None,
        namespace: Optional[SomeSubstitutionsType] = None,
        exec_name: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[SomeParameters] = None,
        remappings: Optional[SomeRemapRules] = None,
        ros_arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
        to_cloud = False,
        **kwargs
    ) -> None:
        """
        Construct an Node action.

        Many arguments are passed eventually to
        :class:`launch.actions.ExecuteProcess`, so see the documentation of
        that class for additional details.
        However, the `cmd` is not meant to be used, instead use the
        `executable` and `arguments` keyword arguments to this function.

        This action, once executed, delegates most work to the
        :class:`launch.actions.ExecuteProcess`, but it also converts some ROS
        specific arguments into generic command line arguments.

        The launch_ros.substitutions.ExecutableInPackage substitution is used
        to find the executable at runtime, so this Action also raise the
        exceptions that substitution can raise when the package or executable
        are not found.

        If the name is not given (or is None) then no name is passed to
        the node on creation and instead the default name specified within the
        code of the node is used instead.

        The namespace can either be absolute (i.e. starts with /) or
        relative.
        If absolute, then nothing else is considered and this is passed
        directly to the node to set the namespace.
        If relative, the namespace in the 'ros_namespace' LaunchConfiguration
        will be prepended to the given relative node namespace.
        If no namespace is given, then the default namespace `/` is
        assumed.

        The parameters are passed as a list, with each element either a yaml
        file that contains parameter rules (string or pathlib.Path to the full
        path of the file), or a dictionary that specifies parameter rules.
        Keys of the dictionary can be strings or an iterable of Substitutions
        that will be expanded to a string.
        Values in the dictionary can be strings, integers, floats, or tuples
        of Substitutions that will be expanded to a string.
        Additionally, values in the dictionary can be lists of the
        aforementioned types, or another dictionary with the same properties.
        A yaml file with the resulting parameters from the dictionary will be
        written to a temporary file, the path to which will be passed to the
        node.
        Multiple parameter dictionaries/files can be passed: each file path
        will be passed in in order to the node (where the last definition of
        a parameter takes effect).
        However, fully qualified node names override wildcards even when
        specified earlier.
        If `namespace` is not specified, dictionaries are prefixed by a
        wildcard namespace (`/**`) and other specific parameter declarations
        may overwrite it.

        Using `ros_arguments` is equivalent to using `arguments` with a
        prepended '--ros-args' item.

        :param: executable the name of the executable to find if a package
            is provided or otherwise a path to the executable to run.
        :param: package the package in which the node executable can be found
        :param: name the name of the node
        :param: namespace the ROS namespace for this Node
        :param: exec_name the label used to represent the process.
            Defaults to the basename of node executable.
        :param: parameters list of names of yaml files with parameter rules,
            or dictionaries of parameters.
        :param: remappings ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: ros_arguments list of ROS arguments for the node
        :param: arguments list of extra arguments for the node
        """
        if package is not None:
            cmd = [ExecutableInPackage(package=package, executable=executable)]
        else:
            cmd = [executable]
        cmd += [] if arguments is None else arguments
        cmd += [] if ros_arguments is None else ['--ros-args'] + ros_arguments
        # Reserve space for ros specific arguments.
        # The substitutions will get expanded when the action is executed.
        cmd += ['--ros-args']  # Prepend ros specific arguments with --ros-args flag
        if name is not None:
            cmd += ['-r', LocalSubstitution(
                "ros_specific_arguments['name']", description='node name')]
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            # All elements in the list are paths to files with parameters (or substitutions that
            # evaluate to paths), or dictionaries of parameters (fields can be substitutions).
            normalized_params = normalize_parameters(parameters)
        # Forward 'exec_name' as to ExecuteProcess constructor
        kwargs['name'] = exec_name
        super().__init__(cmd=cmd, **kwargs)
        self.__package = package
        self.__node_executable = executable
        self.__node_name = name
        self.__node_namespace = namespace
        self.__parameters = [] if parameters is None else normalized_params
        self.__remappings = [] if remappings is None else list(normalize_remap_rules(remappings))
        self.__ros_arguments = ros_arguments
        self.__arguments = arguments

        self.__expanded_node_name = self.UNSPECIFIED_NODE_NAME
        self.__expanded_node_namespace = self.UNSPECIFIED_NODE_NAMESPACE
        self.__expanded_parameter_arguments = None  # type: Optional[List[Tuple[Text, bool]]]
        self.__final_node_name = None  # type: Optional[Text]
        self.__expanded_remappings = None  # type: Optional[List[Tuple[Text, Text]]]

        self.__substitutions_performed = False
        self.__to_cloud = to_cloud
        self.__logger = launch.logging.get_logger(__name__)

        self.__extensions = get_extensions(self.__logger)

    @staticmethod
    def parse_nested_parameters(params, parser):
        """Normalize parameters as expected by Node constructor argument."""
        from ..descriptions import ParameterValue

        def get_nested_dictionary_from_nested_key_value_pairs(params):
            """Convert nested params in a nested dictionary."""
            param_dict = {}
            for param in params:
                name = tuple(parser.parse_substitution(param.get_attr('name')))
                type_identifier = param.get_attr('type', data_type=None, optional=True)
                data_type = None
                if type_identifier is not None:
                    data_type = get_data_type_from_identifier(type_identifier)
                value = param.get_attr('value', data_type=data_type, optional=True)
                nested_params = param.get_attr('param', data_type=List[Entity], optional=True)
                param.assert_entity_completely_parsed()
                if value is not None and nested_params:
                    raise RuntimeError(
                        'nested parameters and value attributes are mutually exclusive')
                if data_type is not None and nested_params:
                    raise RuntimeError(
                        'nested parameters and type attributes are mutually exclusive')
                elif value is not None:
                    some_value = parser.parse_if_substitutions(value)
                    param_dict[name] = ParameterValue(some_value, value_type=data_type)
                elif nested_params:
                    param_dict.update({
                        name: get_nested_dictionary_from_nested_key_value_pairs(nested_params)
                    })
                else:
                    raise RuntimeError('either a value attribute or nested params are needed')
            return param_dict

        normalized_params = []
        for param in params:
            from_attr = param.get_attr('from', optional=True)
            allow_substs = param.get_attr('allow_substs', data_type=bool, optional=True)
            name = param.get_attr('name', optional=True)
            if from_attr is not None and name is not None:
                raise RuntimeError('name and from attributes are mutually exclusive')
            elif from_attr is not None:
                # 'from' attribute ignores 'name' attribute,
                # it's not accepted to be nested,
                # and it can not have children.
                if isinstance(allow_substs, str):
                    allow_substs = parser.parse_substitution(allow_substs)
                else:
                    allow_substs = bool(allow_substs)
                param.assert_entity_completely_parsed()
                normalized_params.append(
                    ParameterFile(parser.parse_substitution(from_attr), allow_substs=allow_substs))
                continue
            elif name is not None:
                if allow_substs is not None:
                    raise RuntimeError(
                        "'allow_substs' can only be used together with 'from' attribute")
                normalized_params.append(
                    get_nested_dictionary_from_nested_key_value_pairs([param]))
                continue
            raise ValueError('param Entity should have name or from attribute')
        return normalized_params

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse node."""
        # See parse method of `ExecuteProcess`
        _, kwargs = super().parse(entity, parser, ignore=['cmd'])
        args = entity.get_attr('args', optional=True)
        if args is not None:
            kwargs['arguments'] = super()._parse_cmdline(args, parser)
        ros_args = entity.get_attr('ros_args', optional=True)
        if ros_args is not None:
            kwargs['ros_arguments'] = super()._parse_cmdline(ros_args, parser)
        node_name = entity.get_attr('node-name', optional=True)
        if node_name is not None:
            kwargs['node_name'] = parser.parse_substitution(node_name)
        node_name = entity.get_attr('name', optional=True)
        if node_name is not None:
            kwargs['name'] = parser.parse_substitution(node_name)
        exec_name = entity.get_attr('exec_name', optional=True)
        if exec_name is not None:
            kwargs['exec_name'] = parser.parse_substitution(exec_name)
        package = entity.get_attr('pkg', optional=True)
        if package is not None:
            kwargs['package'] = parser.parse_substitution(package)
        kwargs['executable'] = parser.parse_substitution(entity.get_attr('exec'))
        ns = entity.get_attr('namespace', optional=True)
        if ns is not None:
            kwargs['namespace'] = parser.parse_substitution(ns)
        remappings = entity.get_attr('remap', data_type=List[Entity], optional=True)
        if remappings is not None:
            kwargs['remappings'] = [
                (
                    parser.parse_substitution(remap.get_attr('from')),
                    parser.parse_substitution(remap.get_attr('to'))
                ) for remap in remappings
            ]
            for remap in remappings:
                remap.assert_entity_completely_parsed()
        parameters = entity.get_attr('param', data_type=List[Entity], optional=True)
        if parameters is not None:
            kwargs['parameters'] = cls.parse_nested_parameters(parameters, parser)

        return cls, kwargs

    @property
    def node_package(self):
        """Getter for node_package."""
        return self.__package

    @property
    def node_executable(self):
        """Getter for node_executable."""
        return self.__node_executable

    @property
    def to_cloud(self):
        """Getter for to_cloud."""
        return self.__to_cloud

    @property
    def node_name(self):
        """Getter for node_name."""
        if self.__final_node_name is None:
            raise RuntimeError("cannot access 'node_name' before executing action")
        return self.__final_node_name

    def is_node_name_fully_specified(self):
        keywords = (self.UNSPECIFIED_NODE_NAME, self.UNSPECIFIED_NODE_NAMESPACE)
        return all(x not in self.node_name for x in keywords)

    def _create_params_file_from_dict(self, params):
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            param_dict = {
                self.node_name if self.is_node_name_fully_specified() else '/**':
                {'ros__parameters': params}
            }
            yaml.dump(param_dict, h, default_flow_style=False)
            return param_file_path

    def _get_parameter_rule(self, param: 'Parameter', context: LaunchContext):
        name, value = param.evaluate(context)
        return f'{name}:={yaml.dump(value)}'

    def _perform_substitutions(self, context: LaunchContext) -> None:
        # Here to avoid cyclic import
        from ..descriptions import Parameter
        try:
            if self.__substitutions_performed:
                # This function may have already been called by a subclass' `execute`, for example.
                return
            self.__substitutions_performed = True
            if self.__node_name is not None:
                self.__expanded_node_name = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_name))
                validate_node_name(self.__expanded_node_name)
            self.__expanded_node_name.lstrip('/')
            expanded_node_namespace: Optional[Text] = None
            if self.__node_namespace is not None:
                expanded_node_namespace = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_namespace))
            base_ns = context.launch_configurations.get('ros_namespace', None)
            expanded_node_namespace = make_namespace_absolute(
                prefix_namespace(base_ns, expanded_node_namespace))
            if expanded_node_namespace is not None:
                self.__expanded_node_namespace = expanded_node_namespace
                cmd_extension = ['-r', LocalSubstitution("ros_specific_arguments['ns']")]
                self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])
                validate_namespace(self.__expanded_node_namespace)
        except Exception:
            self.__logger.error(
                "Error while expanding or validating node name or namespace for '{}':"
                .format('package={}, executable={}, name={}, namespace={}'.format(
                    self.__package,
                    self.__node_executable,
                    self.__node_name,
                    self.__node_namespace,
                ))
            )
            raise
        self.__final_node_name = prefix_namespace(
            self.__expanded_node_namespace, self.__expanded_node_name)

        # Expand global parameters first,
        # so they can be overridden with specific parameters of this Node
        # The params_container list is expected to contain name-value pairs (tuples)
        # and/or strings representing paths to parameter files.
        params_container = context.launch_configurations.get('global_params', None)

        if any(x is not None for x in (params_container, self.__parameters)):
            self.__expanded_parameter_arguments = []
        if params_container is not None:
            for param in params_container:
                if isinstance(param, tuple):
                    name, value = param
                    cmd_extension = ['-p', f'{name}:={value}']
                    self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])
                else:
                    param_file_path = os.path.abspath(param)
                    self.__expanded_parameter_arguments.append((param_file_path, True))
                    cmd_extension = ['--params-file', f'{param_file_path}']
                    assert os.path.isfile(param_file_path)
                    self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])

        # expand parameters too
        if self.__parameters is not None:
            evaluated_parameters = evaluate_parameters(context, self.__parameters)
            for params in evaluated_parameters:
                is_file = False
                if isinstance(params, dict):
                    param_argument = self._create_params_file_from_dict(params)
                    is_file = True
                    assert os.path.isfile(param_argument)
                elif isinstance(params, pathlib.Path):
                    param_argument = str(params)
                    is_file = True
                elif isinstance(params, Parameter):
                    param_argument = self._get_parameter_rule(params, context)
                else:
                    raise RuntimeError('invalid normalized parameters {}'.format(repr(params)))
                if is_file and not os.path.isfile(param_argument):
                    self.__logger.warning(
                        'Parameter file path is not a file: {}'.format(param_argument),
                    )
                    continue
                self.__expanded_parameter_arguments.append((param_argument, is_file))
                cmd_extension = ['--params-file' if is_file else '-p', f'{param_argument}']
                self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])
        # expand remappings too
        global_remaps = context.launch_configurations.get('ros_remaps', None)
        if global_remaps or self.__remappings:
            self.__expanded_remappings = []
        if global_remaps:
            self.__expanded_remappings.extend(global_remaps)
        if self.__remappings:
            self.__expanded_remappings.extend([
                (perform_substitutions(context, src), perform_substitutions(context, dst))
                for src, dst in self.__remappings
            ])
        if self.__expanded_remappings:
            cmd_extension = []
            for src, dst in self.__expanded_remappings:
                cmd_extension.extend(['-r', f'{src}:={dst}'])
            self.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd_extension])

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)
        # Prepare the ros_specific_arguments list and add it to the context so that the
        # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
        ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
        if self.__node_name is not None:
            ros_specific_arguments['name'] = '__node:={}'.format(self.__expanded_node_name)
        if self.__expanded_node_namespace != '':
            ros_specific_arguments['ns'] = '__ns:={}'.format(self.__expanded_node_namespace)

        # Give extensions a chance to prepare for execution
        for extension in self.__extensions.values():
            cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
                context,
                ros_specific_arguments,
                self
            )
            self.cmd.extend(cmd_extension)

        context.extend_locals({'ros_specific_arguments': ros_specific_arguments})
        ret = super().execute(context)

        if self.is_node_name_fully_specified():
            add_node_name(context, self.node_name)
            node_name_count = get_node_name_count(context, self.node_name)
            if node_name_count > 1:
                execute_process_logger = launch.logging.get_logger(self.name)
                execute_process_logger.warning(
                    'there are now at least {} nodes with the name {} created within this '
                    'launch context'.format(node_name_count, self.node_name)
                )

        return ret

    @property
    def expanded_node_namespace(self):
        """Getter for expanded_node_namespace."""
        return self.__expanded_node_namespace

    @property
    def expanded_remapping_rules(self):
        """Getter for expanded_remappings."""
        return self.__expanded_remappings


def instantiate_extension(
    group_name,
    extension_name,
    extension_class,
    extensions,
    logger,
    *,
    unique_instance=False
):
    if not unique_instance and extension_class in extensions:
        return extensions[extension_name]
    try:
        extension_instance = extension_class()
    except plugin_support.PluginException as e:  # noqa: F841
        logger.warning(
            f"Failed to instantiate '{group_name}' extension "
            f"'{extension_name}': {e}")
        return None
    except Exception as e:  # noqa: F841
        logger.error(
            f"Failed to instantiate '{group_name}' extension "
            f"'{extension_name}': {e}")
        return None
    if not unique_instance:
        extensions[extension_name] = extension_instance
    return extension_instance


def get_extensions(logger):
    group_name = 'launch_ros.node_action'
    entry_points = {}
    for entry_point in importlib_metadata.entry_points().get(group_name, []):
        entry_points[entry_point.name] = entry_point
    extension_types = {}
    for entry_point in entry_points:
        try:
            extension_type = entry_points[entry_point].load()
        except Exception as e:  # noqa: F841
            logger.warning(f"Failed to load entry point '{entry_points[entry_point].name}': {e}")
            continue
        extension_types[entry_points[entry_point].name] = extension_type

    extensions = {}
    for extension_name, extension_class in extension_types.items():
        extension_instance = instantiate_extension(
            group_name, extension_name, extension_class, extensions, logger)
        if extension_instance is None:
            continue
        extension_instance.NAME = extension_name
        extensions[extension_name] = extension_instance
    return extensions
