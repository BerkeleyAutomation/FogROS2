from ros2cli.verb import VerbExtension


class ListVerb(VerbExtension):
    def main(self, *, args):
        print("hello world")