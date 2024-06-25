import sys


# CommandParser Start


class CommandParser():

    def __init__(self):

        self.program_name   = None
        self.arguments      = None
        self.count          = 0


    def run(self):
        
        self.program_name   = sys.argv[0]
        self.arguments      = sys.argv[1:]
        self.count          = len(self.arguments)


# CommandParser End



if __name__ == '__main__':

    parser = CommandParser()

    parser.run()
