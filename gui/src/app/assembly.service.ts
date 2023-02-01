import { Injectable } from '@angular/core';
import { Assembly } from './assembly.model';
import { HttpClient, HttpEventType } from '@angular/common/http';

@Injectable({
  providedIn: 'root'
})
export class AssemblyService {

  public assembly: Assembly;

  constructor(private http: HttpClient) {
    // Initialization of assembly object for Pick and Place assemblies
    this.assembly = {
      assembly_name : '',
      assembly_type : '',
      objects : [{name: '', static: false, display_name: ''}],
      object_number : 0,
      keyframes: [
        {semantics: "initial",
        moving_part: 0,
        object_poses: {'test':'test'}},
        {semantics: "grasp_a",
        moving_part: 0,
        object_poses: {}},
        {semantics: "pickup_a",
        moving_part: 0,
        object_poses: {}},
        {semantics: "place_a",
        moving_part: 0,
        object_poses: {}},
        {semantics: "grasp_b",
        moving_part: 0,
        object_poses: {}},
        {semantics: "pickup_b",
        moving_part: 0,
        object_poses: {}},
        {semantics: "place_b",
        moving_part: 0,
        object_poses: {}}
      ]
    }
  }

  uploadJson(file: Assembly) {
    return this.http.post<any>("http://localhost:5000/json", file);
  }

  uploadModel(data) {
    return this.http.post("http://localhost:5000/upload", data, {
      reportProgress: true,
      observe: 'events'
    });
  }

  downloadJson() {
    return this.http.get<any>("http://localhost:5000/download");
  }
}
