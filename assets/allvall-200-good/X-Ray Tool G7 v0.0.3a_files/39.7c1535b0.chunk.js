(this["webpackJsonpstreamlit-browser"]=this["webpackJsonpstreamlit-browser"]||[]).push([[39],{3881:function(e,t,n){"use strict";n.r(t),n.d(t,"default",(function(){return F}));var a=n(10),i=n(6),r=n(9),o=n(105),l=n(7),s=n(8),u=n(0),d=n.n(u),c=n(39),m=n(3897),p=n(37),h=n(2903),f=n(188),b=n(20),g=n(42),v=n(83),y=n.n(v),T=n(137),j=n(138),x=n(69),V=n(13),k=n.n(V),O=n(32),w=k()("div",{target:"e88czh80"})((function(e){var t=e.isDisabled,n=e.theme;return{alignItems:"center",backgroundColor:t?n.colors.gray:n.colors.primary,borderTopLeftRadius:"100%",borderTopRightRadius:"100%",borderBottomLeftRadius:"100%",borderBottomRightRadius:"100%",borderTopStyle:"none",borderBottomStyle:"none",borderRightStyle:"none",borderLeftStyle:"none",boxShadow:"none",display:"flex",height:n.radii.xl,justifyContent:"center",width:n.radii.xl,":focus":{boxShadow:"0 0 0 0.2rem ".concat(Object(O.transparentize)(n.colors.primary,.5)),outline:"none"}}}),""),R=k()("div",{target:"e88czh81"})((function(e){var t=e.isDisabled,n=e.theme;return{fontFamily:n.fonts.monospace,fontSize:n.fontSizes.sm,paddingBottom:n.spacing.twoThirdsSmFont,color:t?n.colors.gray:n.colors.primary,top:"-22px",position:"absolute",whiteSpace:"nowrap",backgroundColor:n.colors.transparent,lineHeight:n.lineHeights.base,fontWeight:"normal"}}),""),S=k()("div",{target:"e88czh82"})((function(e){var t=e.theme;return{paddingBottom:t.spacing.none,paddingLeft:t.spacing.none,paddingRight:t.spacing.none,paddingTop:t.spacing.twoThirdsSmFont,justifyContent:"space-between",alignItems:"center",display:"flex"}}),""),D=k()("div",{target:"e88czh83"})((function(e){var t=e.theme;return{lineHeight:t.lineHeights.base,fontWeight:"normal",fontSize:t.fontSizes.sm,fontFamily:t.fonts.monospace}}),""),C=n(5),B=function(e){Object(l.a)(n,e);var t=Object(s.a)(n);function n(e){var r;return Object(i.a)(this,n),(r=t.call(this,e)).formClearHelper=new f.b,r.state=void 0,r.sliderRef=d.a.createRef(),r.thumbValueRef=d.a.createRef(),r.commitWidgetValueDebounced=void 0,r.commitWidgetValue=function(e){r.props.widgetMgr.setDoubleArrayValue(r.props.element,r.state.value,e)},r.onFormCleared=function(){r.setState({value:r.props.element.default},(function(){return r.commitWidgetValue({fromUi:!0})}))},r.handleChange=function(e){var t=e.value;r.setState({value:t},(function(){return r.commitWidgetValueDebounced({fromUi:!0})}))},r.renderThumb=d.a.forwardRef((function(e,t){var n=e.$value,i=e.$thumbIndex,o=r.formatValue(n[i]),l=Object(c.pick)(e,["role","style","aria-valuemax","aria-valuemin","aria-valuenow","tabIndex","onKeyUp","onKeyDown","onMouseEnter","onMouseLeave","draggable"]);return r.props.element.options.length>0||r.isDateTimeType(),r.thumbValueAlignment(),Object(C.jsx)(w,Object(a.a)(Object(a.a)({},l),{},{isDisabled:e.$disabled,ref:t,"aria-valuetext":o,children:Object(C.jsx)(R,{className:"StyledThumbValue","data-testid":"stThumbValue",isDisabled:e.$disabled,ref:r.thumbValueRef,children:o})}))})),r.renderTickBar=function(){var e=r.props.element,t=e.max,n=e.min;return Object(C.jsxs)(S,{"data-testid":"stTickBar",children:[Object(C.jsx)(D,{"data-testid":"stTickBarMin",children:r.formatValue(n)}),Object(C.jsx)(D,{"data-testid":"stTickBarMax",children:r.formatValue(t)})]})},r.render=function(){var e=r.props,t=e.disabled,n=e.element,i=e.theme,o=e.width,l=e.widgetMgr,s=i.colors,u=i.fonts,d=i.fontSizes,c=i.spacing,p={width:o};return r.formClearHelper.manageFormClearListener(l,n.formId,r.onFormCleared),Object(C.jsxs)("div",{ref:r.sliderRef,className:"stSlider",style:p,children:[Object(C.jsx)(T.d,{label:n.label,children:n.help&&Object(C.jsx)(T.b,{children:Object(C.jsx)(j.a,{content:n.help,placement:x.b.TOP_RIGHT})})}),Object(C.jsx)(m.a,{min:n.min,max:n.max,step:n.step,value:r.value,onChange:r.handleChange,disabled:t,overrides:{Root:{style:{paddingTop:c.twoThirdsSmFont}},Thumb:r.renderThumb,Tick:{style:{fontFamily:u.monospace,fontSize:d.sm}},Track:{style:{paddingBottom:0,paddingLeft:0,paddingRight:0,paddingTop:c.twoThirdsSmFont}},InnerTrack:{style:function(e){var t=e.$disabled;return Object(a.a)({height:"4px"},t?{background:s.transparentDarkenedBgMix60}:{})}},TickBar:r.renderTickBar}})]})},r.commitWidgetValueDebounced=Object(g.a)(200,r.commitWidgetValue.bind(Object(o.a)(r))),r.state={value:r.initialValue},r}return Object(r.a)(n,[{key:"initialValue",get:function(){var e=this.props.widgetMgr.getDoubleArrayValue(this.props.element);return void 0!==e?e:this.props.element.default}},{key:"componentDidMount",value:function(){this.thumbValueAlignment(),this.props.element.setValue?this.updateFromProtobuf():this.commitWidgetValue({fromUi:!1})}},{key:"componentDidUpdate",value:function(){this.maybeUpdateFromProtobuf()}},{key:"componentWillUnmount",value:function(){this.formClearHelper.disconnect()}},{key:"maybeUpdateFromProtobuf",value:function(){this.props.element.setValue&&this.updateFromProtobuf()}},{key:"updateFromProtobuf",value:function(){var e=this,t=this.props.element.value;this.props.element.setValue=!1,this.setState({value:t},(function(){e.commitWidgetValue({fromUi:!1})}))}},{key:"value",get:function(){var e=this.props.element,t=e.min,n=e.max,a=this.state.value,i=a[0],r=a.length>1?a[1]:a[0];return i>r&&(i=r),i<t&&(i=t),i>n&&(i=n),r<t&&(r=t),r>n&&(r=n),a.length>1?[i,r]:[i]}},{key:"isDateTimeType",value:function(){var e=this.props.element.dataType;return e===b.q.DataType.DATETIME||e===b.q.DataType.DATE||e===b.q.DataType.TIME}},{key:"formatValue",value:function(e){var t=this.props.element,n=t.format,a=t.options;return this.isDateTimeType()?y()(e/1e3).format(n):a.length>0?Object(h.sprintf)(n,a[e]):Object(h.sprintf)(n,e)}},{key:"thumbValueAlignment",value:function(){var e=this.sliderRef.current,t=this.thumbValueRef.current;if(e&&t){var n=e.getBoundingClientRect(),a=t.getBoundingClientRect();t.style.left=a.left<n.left?"0":"",t.style.right=a.right>n.right?"0":""}}}]),n}(d.a.PureComponent),F=Object(p.withTheme)(B)}}]);