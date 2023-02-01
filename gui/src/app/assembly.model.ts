export class Assembly {
  assembly_name: string;
  assembly_type: string;
  object_number: number;
  objects: {
    name: string;
    static: boolean;
    display_name: string;
  }[]
  keyframes: {
    semantics: string;
    moving_part: number;
    object_poses: any;
  }[]
}
