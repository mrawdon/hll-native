#Another Native Bloom Filter Wrapper
Wraps the libbf and libbloom libraries
* https://github.com/mavam/libbf
* https://github.com/jvirkki/libbloom


```
//1st argument is error rate second arg is expected # items
const filter = new bff(0.001, 100000);
console.log(filter.lookup("a"))//false
filter.add("a");
console.log(filter.lookup("a"))//true
```

## Notes
Currently libbloom is enabled to swap implementations uncomment line 3 of bloom-napi.h
The error rate may need to be flipped when swapping implementations
libbf has a max_obj_size currently defined at 36