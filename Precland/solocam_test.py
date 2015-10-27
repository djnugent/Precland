import solocam
import sys

ctx = solocam.SoloCam()

ctx.start()

frame = ctx.read_frame()
sys.stdout.write("P5 %u %u 255\n" % (ctx.width, ctx.height))
sys.stdout.write(frame)

ctx.stop()