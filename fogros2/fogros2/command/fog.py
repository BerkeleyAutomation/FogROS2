from ros2cli.command import CommandExtension, add_subparsers_on_demand


class FogCommand(CommandExtension):
    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(parser, cli_name, "_verb", "fogros2.verb", required=False)

    def main(self, *, parser, args):
        if not hasattr(args, "_verb"):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_verb")
        # call the verb's main method
        return extension.main(args=args)
