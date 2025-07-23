# Inspired by www.tinygrad.org

from typing import Dict, Tuple, Union, List, ClassVar, Optional, Iterable, Any, TypeVar, TYPE_CHECKING, Callable, Sequence
import os
import contextlib

def merge_dicts(ds):
    kvs = set([(k,v) for d in ds for k,v in d.items()])
    assert len(kvs) == len(set(kv[0] for kv in kvs)), f"cannot merge, {kvs} contains different values for the same key"
    return {k:v for d in ds for k,v in d.items()}

def getenv(key:str, default=0): return type(default)(os.getenv(key, default))

class GlobalCounters:
    global_ops: ClassVar[int] = 0
    global_mem: ClassVar[int] = 0
    time_sum_s: ClassVar[float] = 0.0
    kernel_count: ClassVar[int] = 0
    mem_used: ClassVar[int] = 0  
    @staticmethod
    def reset(): 
        GlobalCounters.global_ops, GlobalCounters.global_mem, GlobalCounters.time_sum_s, GlobalCounters.kernel_count = 0,0,0.0,0


class ContextVar:
    _cache: ClassVar = {}
    value: int
    key: str
    def __init__(self, key, default_value):
        assert key not in ContextVar._cache, f"attempt to recreate ContextVar {key}"
        ContextVar._cache[key] = self
        self.value, self.key = getenv(key, default_value), key
    def __bool__(self): return bool(self.value)
    def __ge__(self, x): return self.value >= x
    def __gt__(self, x): return self.value > x
    def __lt__(self, x): return self.value < x

DEBUG = ContextVar("DEBUG", 0)

# **************** timer and profiler ****************

class Timing(contextlib.ContextDecorator):
    def __init__(self, prefix="", on_exit=None, enabled=True): self.prefix, self.on_exit, self.enabled = prefix, on_exit, enabled
    def __enter__(self): self.st = time.perf_counter_ns()
    def __exit__(self, *exc):
        self.et = time.perf_counter_ns() - self.st
        if self.enabled: print(f"{self.prefix}{self.et*1e-6:6.2f} ms"+(self.on_exit(self.et) if self.on_exit else ""))

def _format_fcn(fcn): return f"{fcn[0]}:{fcn[1]}:{fcn[2]}"

class Profiling(contextlib.ContextDecorator):
    def __init__(self, enabled=True, sort='cumtime', frac=0.2, fn=None, ts=1):
        self.enabled, self.sort, self.frac, self.fn, self.time_scale = enabled, sort, frac, fn, 1e3/ts
    def __enter__(self):
        import cProfile
        self.pr = cProfile.Profile()
        if self.enabled: self.pr.enable()
    def __exit__(self, *exc):
        if self.enabled:
            self.pr.disable()
            if self.fn: self.pr.dump_stats(self.fn)
            import pstats
            stats = pstats.Stats(self.pr).strip_dirs().sort_stats(self.sort)
            for fcn in stats.fcn_list[0:int(len(stats.fcn_list)*self.frac)]:    # type: ignore[attr-defined]
                (_primitive_calls, num_calls, tottime, cumtime, callers) = stats.stats[fcn]    # type: ignore[attr-defined]
                scallers = sorted(callers.items(), key=lambda x: -x[1][2])
                print(f"n:{num_calls:8d}  tm:{tottime*self.time_scale:7.2f}ms  tot:{cumtime*self.time_scale:7.2f}ms",
                    colored(_format_fcn(fcn).ljust(50), "yellow"),
                    colored(f"<- {(scallers[0][1][2]/tottime)*100:3.0f}% {_format_fcn(scallers[0][0])}", "BLACK") if scallers else '')