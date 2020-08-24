{
  "targets": [
    {
      "target_name": "hll",
      "cflags!": [ "--std=c++11 -fno-exceptions -fno-strict-aliasing" ],
      "cflags_cc!": [ "--std=c++11 -fno-exceptions -fno-strict-aliasing" ],
      "include_dirs": [
        "./src",
        "<!@(node -p \"require('node-addon-api').include\")"
        
      ],
      "dependencies": ["<!(node -p \"require('node-addon-api').gyp\")"],
      'defines': [ 'NAPI_DISABLE_CPP_EXCEPTIONS' ],
      "sources": [
        "<!@(node -p \"require('fs').readdirSync('./src').map(f=>'src/'+f).join(' ')\")"
      ],
    }
  ]
}
