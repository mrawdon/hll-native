export default class HllNative{
    constructor();
    add(data: string):void;
    merge(hll: HllNative):void;
    cardinality():number;
    deserialize(date:string):void;
}
