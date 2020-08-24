declare class BloomFilterNative{
    constructor(errorRate: number, numberFiles: number);
    add(data: string):void;
    lookup(data: string):boolean;
}

declare module "bloom-filter-native"{
    export = BloomFilterNative
}