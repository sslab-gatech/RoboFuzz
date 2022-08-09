from tracer import APITracer

t = APITracer()
d = t.parse_trace(filename="trace-0")
print(d)
