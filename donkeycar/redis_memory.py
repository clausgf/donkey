#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import redis

class RedisMemory:
    """
    A convenience class to save key/value pairs in redis - compatible to the
    default Memory class.
    """
    def __init__(self, host="localhost", port=6379, *args, **kw):
        self.host = host
        self.port = port
        self.redis = redis.StrictRedis(self.host, self.port)
        self.d = {}

    def __setitem__(self, key, value):
        if type(key) is not tuple:
            key = (key,)
            value=(value,)

        pipe = self.redis.pipeline()
        for i, k in enumerate(key):
            pipe.set(k, value[i])
            self.d[k] = value[i]
        pipe.execute()

    def __getitem__(self, key):
        if type(key) is tuple:
            return [self.d[k] for k in key]
        else:
            return self.d[key]

    def update(self, new_d):
        pipe = self.redis.pipeline()
        for key, value in new_d.items():
            pipe.set(key, value)
        pipe.execute()
        self.d.update(new_d)

    def put(self, keys, inputs):
        if len(keys) > 1:
            pipe = self.redis.pipeline()
            for i, key in enumerate(keys):
                pipe.set(key, inputs[i])
                try:
                    self.d[key] = inputs[i]
                except IndexError as e:
                    error = str(e) + ' issue with keys: ' + str(key)
                    raise IndexError(error)
            pipe.execute()
        else:
            self.redis.set(keys[0], inputs)
            self.d[keys[0]] = inputs

    def get(self, keys):
        result = [self.d.get(k) for k in keys]
        return result

    def keys(self):
        return self.d.keys()

    def values(self):
        return self.d.values()

    def items(self):
        return self.d.items()
