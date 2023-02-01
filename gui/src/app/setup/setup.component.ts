import { Component, OnInit } from '@angular/core';
import { HttpClient, HttpEventType } from '@angular/common/http';
import { AssemblyService } from '../assembly.service';
import { Router } from '@angular/router';

@Component({
  selector: 'app-setup',
  templateUrl: './setup.component.html',
  styleUrls: ['./setup.component.css']
})
export class SetupComponent implements OnInit {

  assemblyType: string = "";
  assemblyName: string = "";
  assemblyTypes = [{name: "Pick and Place", desc: "Each part is picked and placed on a specific location or on top of each other"},
                   {name: "Simple Insertion", desc: "The parts need to be inserted in a base using special slots"},
                   {name: "Insertion by Deformation", desc: "The parts need to be elastically deformed in order to be inserted"}];
  partAName = "";
  partAModel = '';
  partAStatic = false;
  partBName = "";
  partBModel = '';
  partBStatic = false;
  partCName = "";
  partCModel = '';
  partCStatic = false;
  uploadStatus = [null, null, null];  // An array indicating the upload status for each 3D part
  numParts: string = "";

  constructor(private http: HttpClient, private assService: AssemblyService, private router: Router) { }

  ngOnInit(): void {
    // If already saved assembly use it
    if (this.assService.assembly.assembly_name) this.assemblyName = this.assService.assembly.assembly_name;
    if (this.assService.assembly.assembly_type) this.assemblyType = this.assService.assembly.assembly_type;
    if (this.assService.assembly.object_number>0) this.numParts = this.assService.assembly.object_number.toString();
    if (this.assService.assembly.objects.length>=2) {
      this.partAName = this.assService.assembly.objects[0].name;
      this.partAStatic = this.assService.assembly.objects[0].static;
      this.partBName = this.assService.assembly.objects[1].name;
      this.partBStatic = this.assService.assembly.objects[1].static;
      this.uploadStatus = ['completed', 'completed'];
    }
    if (this.assService.assembly.objects.length==3) {
      this.partCName = this.assService.assembly.objects[2].name;
      this.partCStatic = this.assService.assembly.objects[2].static;
      this.uploadStatus = ['completed', 'completed', 'completed'];
    }

  }

  onFileSelected(event, part: number) {

    const file:File = event.target.files[0];

    if (file) {

      switch (part) {
        case 1:
          this.partAName = file.name.replace(/\.[^/.]+$/, "");
          this.partAModel = file.name.replace(/\.[^/.]+$/, "");
          break;
        case 2:
          this.partBName = file.name.replace(/\.[^/.]+$/, "");
          this.partBModel = file.name.replace(/\.[^/.]+$/, "");
          break;
        case 3:
          this.partCName = file.name.replace(/\.[^/.]+$/, "");
          this.partCModel = file.name.replace(/\.[^/.]+$/, "");
          break;
      }

      const formData = new FormData();

      formData.append("part", file);

      /* const upload = this.http.post("http://160.40.51.95:5000/upload", formData, {
        reportProgress: true,
        observe: 'events'
      }); */

      this.assService.uploadModel(formData).subscribe(event => {
        if (event.type == HttpEventType.UploadProgress) {

          if (typeof event.loaded == 'number') {
            if (event.loaded != event.total) {
              this.uploadStatus[part-1] = 'uploading';
            }
            else {
              this.uploadStatus[part-1] = 'completed';
            }
          }
          else {
            console.log("Error while uploading!");
          }
        }
      }, error => {
        console.log(error);
        this.uploadStatus[part-1] = null;
      });


    }
  }

  checkNames() {
    if (this.numParts==='2') {
      if (this.partAName&&this.partBName&&this.partAName===this.partBName) {
        return true;
      }
      else {
        return false;
      }
    }
    else if (this.numParts==='3') {
      if (this.partAName&&this.partBName&&this.partCName&&(this.partAName===this.partBName||this.partAName===this.partCName||this.partCName===this.partBName)) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }

  onProceed() {
    this.assService.assembly.assembly_name = this.assemblyName;
    this.assService.assembly.assembly_type = this.assemblyType;
    this.assService.assembly.object_number = +this.numParts;
    this.assService.assembly.objects = [{display_name: this.partAName, static: this.partAStatic, name: this.partAModel}, {display_name: this.partBName, static: this.partBStatic, name: this.partBModel}];
    if (this.numParts==="3") this.assService.assembly.objects.push({display_name: this.partCName, static: this.partCStatic, name: this.partCModel});

    this.router.navigate(['/teach']);

  }

  proceedDisabled() {
    if ((this.assemblyName!='')&&(this.assemblyType!='')&&!this.checkNames()) {
      if ((this.partAName!='')&&(this.partBName!='')&&(this.uploadStatus[0]=='completed')&&(this.uploadStatus[0]=='completed')) {
        if (this.numParts=='2') {
          return false;
        }
        else if ((this.numParts=='3')&&(this.partCName!='')&&(this.uploadStatus[2]=='completed')) {
          return false;
        }
        else {
          return true;
        }
      }
      else {
        return true;
      }
    }
    else {
      return true;
    }
  }
}
