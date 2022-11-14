# Copyright 2022 The Regents of the University of California (Regents)
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
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


from builtins import str
from builtins import range
from io import open
import os


class WGConfig():
    SECTION_FIRSTLINE = '_index_firstline'
    SECTION_LASTLINE = '_index_lastline'
    SECTION_RAW = '_rawdata'
    _interface = None  # interface attributes
    _peers = None  # peer data

    def __init__(self, file, keyattr='PublicKey'):
        self.filename = self.file2filename(file)
        self.keyattr = keyattr
        self.lines = []
        self.initialize_file()

    @staticmethod
    def file2filename(file):
        if os.path.basename(file) == file:
            if not file.endswith('.conf'):
                file += '.conf'
            file = os.path.join('/etc/wireguard', file)
        return file

    def invalidate_data(self):
        self._interface = None
        self._peers = None

    def read_file(self):
        with open(self.filename, 'r') as wgfile:
            self.lines = [line.rstrip() for line in wgfile.readlines()]
        self.invalidate_data()

    def write_file(self, file=None):
        if file is None:
            filename = self.filename
        else:
            filename = self.file2filename(file)
        with os.fdopen(
                os.open(filename, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o640), 'w') as wgfile:
            wgfile.writelines(line + '\n' for line in self.lines)

    @staticmethod
    def parse_line(line):
        attr, _, value = line.partition('=')
        attr = attr.strip()
        parts = value.partition('#')
        value = parts[0].strip()  # strip comments and whitespace
        value = str(value)  # this line is for Python2 support only
        comment = parts[1] + parts[2]
        if value.isnumeric():
            value = [int(value)]
        else:
            # decompose into list based on commata as separator
            value = [item.strip() for item in value.split(',')]
        return attr, value, comment

    def parse_lines(self):
        # There will be two special attributes in the parsed data:
        # _index_firstline: Line (zero indexed) of the section header
        # (including any leading lines with comments)
        # _index_lastline: Line (zero indexed) of the last attribute line of the section
        # (including any directly following comments)

        def close_section(section, section_data):
            section_data = {k: (v if len(v) > 1 else v[0]) for k, v in section_data.items()}
            if section is None:  # nothing to close on first section
                return
            elif section == 'interface':  # close interface section
                self._interface = section_data
            else:  # close peer section
                peername = section_data.get(self.keyattr)
                self._peers[peername] = section_data
            section_data[self.SECTION_RAW] = self.lines[section_data[self.SECTION_FIRSTLINE]:
                                                        (section_data[self.SECTION_LASTLINE] + 1)]

        self._interface = dict()
        self._peers = dict()
        section = None
        section_data = dict()
        last_empty_line_in_section = -1  # virtual empty line before start of file
        for i, line in enumerate(self.lines):
            # Ignore leading whitespace and trailing whitespace
            line = line.strip()
            # Ignore empty lines and comments
            if len(line) == 0:
                last_empty_line_in_section = i
                continue
            if line.startswith('['):  # section
                if last_empty_line_in_section is not None:
                    section_data[self.SECTION_LASTLINE] = [last_empty_line_in_section - 1]
                close_section(section, section_data)
                section_data = dict()
                section = line[1:].partition(']')[0].lower()
                if last_empty_line_in_section is None:
                    section_data[self.SECTION_FIRSTLINE] = [i]
                else:
                    section_data[self.SECTION_FIRSTLINE] = [last_empty_line_in_section + 1]
                    last_empty_line_in_section = None
                section_data[self.SECTION_LASTLINE] = [i]
                if section not in ['interface', 'peer']:
                    raise ValueError('Unsupported section [{0}] in line {1}'.format(section, i))
            elif line.startswith('#'):
                section_data[self.SECTION_LASTLINE] = [i]
            else:  # regular line
                attr, value, _comment = self.parse_line(line)
                section_data[attr] = section_data.get(attr, [])
                section_data[attr].extend(value)
                section_data[self.SECTION_LASTLINE] = [i]
        close_section(section, section_data)

    def handle_leading_comment(self, leading_comment):
        if leading_comment is not None:
            if leading_comment.strip()[0] != '#':
                raise ValueError('A comment needs to start with a "#"')
            self.lines.append(leading_comment)

    def initialize_file(self, leading_comment=None):
        self.lines = list()
        self.handle_leading_comment(leading_comment)  # add leading comment if needed
        self.lines.append('[Interface]')
        self.invalidate_data()

    def add_peer(self, key, leading_comment=None):
        if key in self.peers:
            raise KeyError('Peer to be added already exists')
        self.lines.append('')  # append an empty line for separation
        self.handle_leading_comment(leading_comment)  # add leading comment if needed
        # Append peer with key attribute
        self.lines.append('[Peer]')
        self.lines.append('{0} = {1}'.format(self.keyattr, key))
        # Invalidate data cache
        self.invalidate_data()

    def del_peer(self, key):
        if key not in self.peers:
            raise KeyError('The peer to be deleted does not exist')
        section_firstline = self.peers[key][self.SECTION_FIRSTLINE]
        section_lastline = self.peers[key][self.SECTION_LASTLINE]
        # Remove a blank line directly before the peer section
        if section_firstline > 0:
            if len(self.lines[section_firstline - 1]) == 0:
                section_firstline -= 1
        # Only keep needed lines
        result = []
        if section_firstline > 0:
            result.extend(self.lines[0:section_firstline])
        result.extend(self.lines[(section_lastline + 1):])
        self.lines = result
        # Invalidate data cache
        self.invalidate_data()

    def get_sectioninfo(self, key):
        if key is None:  # interface
            section_firstline = self.interface[self.SECTION_FIRSTLINE]
            section_lastline = self.interface[self.SECTION_LASTLINE]
        else:  # peer
            if key not in self.peers:
                raise KeyError('The specified peer does not exist')
            section_firstline = self.peers[key][self.SECTION_FIRSTLINE]
            section_lastline = self.peers[key][self.SECTION_LASTLINE]
        return section_firstline, section_lastline

    def add_attr(self, key, attr, value, leading_comment=None, append_as_line=False):
        section_firstline, section_lastline = self.get_sectioninfo(key)
        if leading_comment is not None:
            if leading_comment.strip()[0] != '#':
                raise ValueError('A comment needs to start with a "#"')
        # Look for line with the attribute
        line_found = None
        for i in range(section_firstline + 1, section_lastline + 1):
            line_attr, line_value, line_comment = self.parse_line(self.lines[i])
            if attr == line_attr:
                line_found = i
        # Add the attribute at the right place
        if (line_found is None) or append_as_line:
            line_found = section_lastline if (line_found is None) else line_found
            line_found += 1
            self.lines.insert(line_found, '{0} = {1}'.format(attr, value))
        else:
            line_attr, line_value, line_comment = self.parse_line(self.lines[line_found])
            line_value.append(value)
            if len(line_comment) > 0:
                line_comment = ' ' + line_comment
            line_value = [str(item) for item in line_value]
            self.lines[line_found] = line_attr + ' = ' + ', '.join(line_value) + line_comment
        # Handle leading comments
        if leading_comment is not None:
            self.lines.insert(line_found, leading_comment)
        # Invalidate data cache
        self.invalidate_data()

    def del_attr(self, key, attr, value=None, remove_leading_comments=True):
        section_firstline, section_lastline = self.get_sectioninfo(key)
        # Find all lines with matching attribute name and (if requested) value
        line_found = []
        for i in range(section_firstline + 1, section_lastline + 1):
            line_attr, line_value, line_comment = self.parse_line(self.lines[i])
            if attr == line_attr:
                if (value is None) or (value in line_value):
                    line_found.append(i)
        if len(line_found) == 0:
            raise ValueError('The attribute/value to be deleted is not present')
        # Process all relevant lines
        for i in reversed(line_found):  # reversed so that non-processed indices stay valid
            if value is None:
                del(self.lines[i])
            else:
                line_attr, line_value, line_comment = self.parse_line(self.lines[i])
                line_value.remove(value)
                if len(line_value) > 0:  # keep remaining values in that line
                    self.lines[i] = line_attr + ' = ' + ', '.join(line_value) + line_comment
                else:  # otherwise line is no longer needed
                    del(self.lines[i])
        # Handle leading comments
        if remove_leading_comments:
            i = line_found[0] - 1
            while i > 0:
                if len(self.lines[i]) and (self.lines[i][0] == '#'):
                    del(self.lines[i])
                    i -= 1
                else:
                    break
        # Invalidate data cache
        self.invalidate_data()

    @property
    def interface(self):
        if self._interface is None:
            self.parse_lines()
        return self._interface

    @property
    def peers(self):
        if self._peers is None:
            self.parse_lines()
        return self._peers
