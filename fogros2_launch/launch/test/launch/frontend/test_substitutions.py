# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Test the default substitution interpolator."""

from typing import List
from typing import Text

from launch import LaunchContext
from launch import SomeSubstitutionsType
from launch import Substitution
from launch.actions import ExecuteProcess
from launch.frontend.expose import expose_substitution
from launch.frontend.parse_substitution import parse_if_substitutions
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
from launch.substitutions import ThisLaunchFileDir

import pytest


def test_no_text():
    subst = parse_substitution('')
    assert len(subst) == 1
    assert subst[0].perform(None) == ''


def test_text_only():
    subst = parse_substitution("'yes'")
    assert len(subst) == 1
    assert subst[0].perform(None) == "'yes'"
    subst = parse_substitution('"yes"')
    assert len(subst) == 1
    assert subst[0].perform(None) == '"yes"'
    subst = parse_substitution('10')
    assert len(subst) == 1
    assert subst[0].perform(None) == '10'
    subst = parse_substitution('10e4')
    assert len(subst) == 1
    assert subst[0].perform(None) == '10e4'
    subst = parse_substitution('10e4')
    assert len(subst) == 1
    assert subst[0].perform(None) == '10e4'


def perform_substitutions_without_context(subs: List[Substitution]):
    return ''.join([sub.perform(None) for sub in subs])


class CustomSubstitution(Substitution):

    def __init__(self, text):
        self.__text = text

    def perform(self, context):
        return self.__text


@expose_substitution('test')
def parse_test_substitution(data):
    if not data or len(data) > 1:
        raise RuntimeError()
    kwargs = {}
    kwargs['text'] = perform_substitutions_without_context(data[0])
    return CustomSubstitution, kwargs


def test_text_with_embedded_substitutions():
    subst = parse_substitution('why_$(test asd)_asdasd_$(test bsd)')
    assert len(subst) == 4
    assert subst[0].perform(None) == 'why_'
    assert subst[1].perform(None) == 'asd'
    assert subst[2].perform(None) == '_asdasd_'
    assert subst[3].perform(None) == 'bsd'


def test_dollar_symbol_not_followed_by_bracket():
    subst = parse_substitution('$0 $1')
    assert len(subst) == 1
    assert subst[0].perform(None) == '$0 $1'
    subst = parse_substitution("$(test '$0 $1')")
    assert len(subst) == 1
    assert subst[0].perform(None) == '$0 $1'

# TODO(ivanpauno): Don't deppend on substitution parsing methods for testing the interpolator.
# Write some dummy substitutions and parsing methods instead.


def test_substitution_with_multiple_arguments():
    subst = parse_substitution('$(env what heck)')
    assert len(subst) == 1
    subst = subst[0]
    assert subst.name[0].perform(None) == 'what'
    assert subst.default_value[0].perform(None) == 'heck'


def test_escaped_characters():
    subst = parse_substitution(r'$(env what/\$\(test asd\\\)) 10 10)')
    assert len(subst) == 2
    assert subst[0].name[0].perform(None) == 'what/$(test'
    assert subst[0].default_value[0].perform(None) == r'asd\)'
    assert subst[1].perform(None) == ' 10 10)'


def test_nested_substitutions():
    subst = parse_substitution('$(env what/$(test asd) 10) 10 10)')
    assert len(subst) == 2
    assert len(subst[0].name) == 2
    assert subst[0].name[0].perform(None) == 'what/'
    assert subst[0].name[1].perform(None) == 'asd'
    assert subst[0].default_value[0].perform(None) == '10'
    assert subst[1].perform(None) == ' 10 10)'


def test_quoted_nested_substitution():
    subst = parse_substitution(
        'go_to_$(env WHERE asd)_of_$(env '
        "'something $(test 10)')"
    )
    assert len(subst) == 4
    assert subst[0].perform(None) == 'go_to_'
    assert subst[1].name[0].perform(None) == 'WHERE'
    assert subst[1].default_value[0].perform(None) == 'asd'
    assert subst[2].perform(None) == '_of_'
    assert subst[3].name[0].perform(None) == 'something '
    assert subst[3].name[1].perform(None) == '10'
    assert subst[3].default_value is None


def test_double_quoted_nested_substitution():
    subst = parse_substitution(
        r'$(env "asd_bsd_qsd_$(test \"asd_bds\")" "$(env DEFAULT)_qsd")'
    )
    context = LaunchContext()
    assert len(subst) == 1
    assert len(subst[0].name) == 2
    assert subst[0].name[0].perform(context) == 'asd_bsd_qsd_'
    assert subst[0].name[1].perform(context) == '"asd_bds"'
    assert len(subst[0].default_value) == 2
    assert subst[0].default_value[0].name[0].perform(context) == 'DEFAULT'
    assert subst[0].default_value[0].default_value is None
    assert subst[0].default_value[1].perform(context) == '_qsd'


def test_combining_quotes_nested_substitution():
    subst = parse_substitution(
        '$(env "asd_bsd_qsd_$(test \'asd_bds\')" \'$(env DEFAULT)_qsd\')'
    )
    context = LaunchContext()
    assert len(subst) == 1
    assert len(subst[0].name) == 2
    assert subst[0].name[0].perform(context) == 'asd_bsd_qsd_'
    assert subst[0].name[1].perform(context) == "'asd_bds'"
    assert len(subst[0].default_value) == 2
    assert subst[0].default_value[0].name[0].perform(context) == 'DEFAULT'
    assert subst[0].default_value[0].default_value is None
    assert subst[0].default_value[1].perform(context) == '_qsd'


def test_dirname_subst():
    subst = parse_substitution('$(dirname)')
    assert len(subst) == 1
    assert isinstance(subst[0], ThisLaunchFileDir)


def test_env_subst():
    subst = parse_substitution('$(env asd bsd)')
    assert len(subst) == 1
    env = subst[0]
    assert isinstance(env, EnvironmentVariable)
    assert 'asd' == perform_substitutions_without_context(env.name)
    assert 'bsd' == perform_substitutions_without_context(env.default_value)
    subst = parse_substitution("$(env asd '')")
    assert len(subst) == 1
    env = subst[0]
    assert isinstance(env, EnvironmentVariable)
    assert 'asd' == perform_substitutions_without_context(env.name)
    assert '' == perform_substitutions_without_context(env.default_value)
    subst = parse_substitution('$(env asd)')
    assert len(subst) == 1
    env = subst[0]
    assert isinstance(env, EnvironmentVariable)
    assert 'asd' == perform_substitutions_without_context(env.name)
    assert env.default_value is None


def test_eval_subst():
    subst = parse_substitution(r'$(eval "\'asd\' + \'bsd\'")')
    assert len(subst) == 1
    expr = subst[0]
    assert isinstance(expr, PythonExpression)
    assert 'asdbsd' == expr.perform(LaunchContext())


def test_eval_subst_of_math_expr():
    subst = parse_substitution(r'$(eval "ceil(1.3)")')
    assert len(subst) == 1
    expr = subst[0]
    assert isinstance(expr, PythonExpression)
    assert '2' == expr.perform(LaunchContext())


def expand_cmd_subs(cmd_subs: List[SomeSubstitutionsType]):
    return [perform_substitutions_without_context(x) for x in cmd_subs]


def test_parse_if_substitutions():
    assert parse_if_substitutions(1) == 1
    assert parse_if_substitutions('asd') == 'asd'

    subst = parse_if_substitutions('$(test asd)')
    assert len(subst) == 1
    assert isinstance(subst[0], CustomSubstitution)

    subst = parse_if_substitutions('[$(test asd), $(test bsd)]')
    assert len(subst) == 5
    assert isinstance(subst[0], TextSubstitution)
    assert isinstance(subst[1], CustomSubstitution)
    assert isinstance(subst[2], TextSubstitution)
    assert isinstance(subst[3], CustomSubstitution)
    assert isinstance(subst[4], TextSubstitution)

    subst = parse_if_substitutions(['$(test asd)', '$(test bsd)'])
    assert len(subst) == 2
    assert len(subst[0]) == 1
    assert isinstance(subst[0][0], CustomSubstitution)
    assert len(subst[1]) == 1
    assert isinstance(subst[1][0], CustomSubstitution)

    subst = parse_if_substitutions(['$(test asd)', 'bsd'])
    assert len(subst) == 2
    assert len(subst[0]) == 1
    assert isinstance(subst[0][0], CustomSubstitution)
    assert subst[1] == 'bsd'

    subst = parse_if_substitutions(['$(test asd)', 1])
    assert len(subst) == 2
    assert len(subst[0]) == 1
    assert isinstance(subst[0][0], CustomSubstitution)
    assert subst[1] == 1

    with pytest.raises(ValueError):
        parse_if_substitutions(['$(test asd)', 1, 1.0])


class MockParser:

    def parse_substitution(self, value: Text) -> SomeSubstitutionsType:
        return parse_substitution(value)


def test_execute_process_parse_cmd_line():
    """Test ExecuteProcess._parse_cmd_line."""
    parser = MockParser()

    cmd_text: Text = '$(test path)/a/b/c asd csd $(test asd)/bsd/csd'
    cmd_subs: List[SomeSubstitutionsType] = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed: List[Text] = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['path/a/b/c', 'asd', 'csd', 'asd/bsd/csd']

    cmd_text = '$(test exec) asd $(test bsd) csd'
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['exec', 'asd', 'bsd', 'csd']

    cmd_text = '$(test exec) prefix/$(test bsd)'
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['exec', 'prefix/bsd']

    cmd_text = '$(test exec) prefix/$(test bsd) '
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['exec', 'prefix/bsd']

    cmd_text = '$(test exec) asd prefix/$(test bsd) '
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['exec', 'asd', 'prefix/bsd']

    cmd_text = 'exec asd prefix/bsd '
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['exec', 'asd', 'prefix/bsd']

    cmd_text = '$(test foo)$(test bar)'
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['foobar']

    cmd_text = '$(test that) $(test this)'
    cmd_subs = ExecuteProcess._parse_cmdline(cmd_text, parser)
    cmd_performed = expand_cmd_subs(cmd_subs)
    assert cmd_performed == ['that', 'this']
