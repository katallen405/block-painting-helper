import smach

class Foo(smach.State):
   2      def __init__(self, outcomes=['outcome1', 'outcome2']):
   3        # Your state initialization goes here
   4 
   5      def execute(self, userdata):
   6         # Your state execution goes here
   7         if xxxx:
   8             return 'outcome1'
   9         else:
  10             return 'outcome2'
