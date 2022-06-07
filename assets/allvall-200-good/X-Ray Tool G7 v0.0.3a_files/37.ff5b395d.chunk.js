(this["webpackJsonpstreamlit-browser"]=this["webpackJsonpstreamlit-browser"]||[]).push([[37],{3872:function(e,t,r){"use strict";r.r(t),r.d(t,"default",(function(){return se}));var o=r(6),n=r(9),i=r(7),a=r(8),s=r(0),c=r.n(s),l=r(17),u=r(37),p=r(18),d=r(28);function f(e,t){var r=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);t&&(o=o.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),r.push.apply(r,o)}return r}function h(e){for(var t=1;t<arguments.length;t++){var r=null!=arguments[t]?arguments[t]:{};t%2?f(Object(r),!0).forEach((function(t){b(e,t,r[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(r)):f(Object(r)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(r,t))}))}return e}function b(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}function m(e){return e.$isActive?2:e.$isHovered?1:0}function v(e){var t=e.$theme.colors,r=e.$disabled,o=e.$checked,n=e.$isFocusVisible,i=e.$error,a=e.$isError;if(r)return t.tickFillDisabled;if(!o)return n?t.borderSelected:i||a?t.tickBorderError:t.tickBorder;if(i||a)switch(m(e)){case 0:return t.tickFillErrorSelected;case 1:return t.tickFillErrorSelectedHover;case 2:return t.tickFillErrorSelectedHoverActive}else switch(m(e)){case 0:return t.tickFillSelected;case 1:return t.tickFillSelectedHover;case 2:return t.tickFillSelectedHoverActive}return null}function g(e){var t=e.$theme.colors;if(e.$disabled)return t.tickMarkFillDisabled;if(e.$checked)return t.tickMarkFill;if(e.$error||e.$isError)switch(m(e)){case 0:return t.tickFillError;case 1:return t.tickFillErrorHover;case 2:return t.tickFillErrorHoverActive}else switch(m(e)){case 0:return t.tickFill;case 1:return t.tickFillHover;case 2:return t.tickFillActive}}function y(e){var t=e.$disabled,r=e.$theme.colors;return t?r.contentSecondary:r.contentPrimary}var O=Object(d.a)("div",(function(e){var t=e.$disabled,r=e.$align;return{display:"flex",flexWrap:"wrap",flexDirection:"horizontal"===r?"row":"column",alignItems:"horizontal"===r?"center":"flex-start",cursor:t?"not-allowed":"pointer","-webkit-tap-highlight-color":"transparent"}}));O.displayName="RadioGroupRoot";var j=Object(d.a)("label",(function(e){var t,r=e.$disabled,o=e.$hasDescription,n=e.$labelPlacement,i=e.$theme,a=e.$align,s=i.sizing,c="horizontal"===a,l="rtl"===i.direction?"Left":"Right";return b(t={flexDirection:"top"===n||"bottom"===n?"column":"row",display:"flex",alignItems:"center",cursor:r?"not-allowed":"pointer",marginTop:s.scale200},"margin".concat(l),c?s.scale200:null),b(t,"marginBottom",o&&!c?null:s.scale200),t}));j.displayName="Root";var w=Object(d.a)("div",(function(e){var t=e.$theme,r=t.animation,o=t.sizing;return{backgroundColor:g(e),borderTopLeftRadius:"50%",borderTopRightRadius:"50%",borderBottomRightRadius:"50%",borderBottomLeftRadius:"50%",height:e.$checked?o.scale200:o.scale550,transitionDuration:r.timing200,transitionTimingFunction:r.easeOutCurve,width:e.$checked?o.scale200:o.scale550}}));w.displayName="RadioMarkInner";var k=Object(d.a)("div",(function(e){var t=e.$theme,r=t.animation,o=t.sizing;return{alignItems:"center",backgroundColor:v(e),borderTopLeftRadius:"50%",borderTopRightRadius:"50%",borderBottomRightRadius:"50%",borderBottomLeftRadius:"50%",boxShadow:e.$isFocusVisible&&e.$checked?"0 0 0 3px ".concat(e.$theme.colors.accent):"none",display:"flex",height:o.scale700,justifyContent:"center",marginTop:o.scale0,marginRight:o.scale0,marginBottom:o.scale0,marginLeft:o.scale0,outline:"none",verticalAlign:"middle",width:o.scale700,flexShrink:0,transitionDuration:r.timing200,transitionTimingFunction:r.easeOutCurve}}));k.displayName="RadioMarkOuter";var F=Object(d.a)("div",(function(e){var t=e.$theme.typography;return h({verticalAlign:"middle"},function(e){var t,r=e.$labelPlacement,o=void 0===r?"":r,n=e.$theme;switch(o){case"top":t="Bottom";break;case"bottom":t="Top";break;case"left":t="rtl"===n.direction?"Left":"Right";break;default:case"right":t="rtl"===n.direction?"Right":"Left"}var i=n.sizing.scale300;return b({},"padding".concat(t),i)}(e),{color:y(e)},t.LabelMedium)}));F.displayName="Label";var $=Object(d.a)("input",{opacity:0,width:0,overflow:"hidden",marginTop:0,marginRight:0,marginBottom:0,marginLeft:0,paddingTop:0,paddingRight:0,paddingBottom:0,paddingLeft:0,position:"absolute"});$.displayName="Input";var R=Object(d.a)("div",(function(e){var t,r=e.$theme,o=e.$align,n="horizontal"===o,i="rtl"===r.direction?"Right":"Left",a="rtl"===r.direction?"Left":"Right";return h({},r.typography.ParagraphSmall,(b(t={color:r.colors.contentSecondary,cursor:"auto"},"margin".concat(i),"horizontal"===o?null:r.sizing.scale900),b(t,"margin".concat(a),n?r.sizing.scale200:null),b(t,"maxWidth","240px"),t))}));R.displayName="Description";var P=r(46);function E(e){return(E="function"===typeof Symbol&&"symbol"===typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"===typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e})(e)}function S(){return(S=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var o in r)Object.prototype.hasOwnProperty.call(r,o)&&(e[o]=r[o])}return e}).apply(this,arguments)}function M(e,t){var r=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);t&&(o=o.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),r.push.apply(r,o)}return r}function x(e){for(var t=1;t<arguments.length;t++){var r=null!=arguments[t]?arguments[t]:{};t%2?M(Object(r),!0).forEach((function(t){I(e,t,r[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(r)):M(Object(r)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(r,t))}))}return e}function C(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){if(!(Symbol.iterator in Object(e))&&"[object Arguments]"!==Object.prototype.toString.call(e))return;var r=[],o=!0,n=!1,i=void 0;try{for(var a,s=e[Symbol.iterator]();!(o=(a=s.next()).done)&&(r.push(a.value),!t||r.length!==t);o=!0);}catch(c){n=!0,i=c}finally{try{o||null==s.return||s.return()}finally{if(n)throw i}}return r}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance")}()}function D(e,t){if(!(e instanceof t))throw new TypeError("Cannot call a class as a function")}function L(e,t){for(var r=0;r<t.length;r++){var o=t[r];o.enumerable=o.enumerable||!1,o.configurable=!0,"value"in o&&(o.writable=!0),Object.defineProperty(e,o.key,o)}}function T(e,t){return!t||"object"!==E(t)&&"function"!==typeof t?V(e):t}function B(e){return(B=Object.setPrototypeOf?Object.getPrototypeOf:function(e){return e.__proto__||Object.getPrototypeOf(e)})(e)}function V(e){if(void 0===e)throw new ReferenceError("this hasn't been initialised - super() hasn't been called");return e}function A(e,t){return(A=Object.setPrototypeOf||function(e,t){return e.__proto__=t,e})(e,t)}function I(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}var H=function(e){function t(){var e,r;D(this,t);for(var o=arguments.length,n=new Array(o),i=0;i<o;i++)n[i]=arguments[i];return I(V(r=T(this,(e=B(t)).call.apply(e,[this].concat(n)))),"state",{isFocusVisible:!1,focusedRadioIndex:-1}),I(V(r),"handleFocus",(function(e,t){Object(P.d)(e)&&r.setState({isFocusVisible:!0}),r.setState({focusedRadioIndex:t}),r.props.onFocus&&r.props.onFocus(e)})),I(V(r),"handleBlur",(function(e,t){!1!==r.state.isFocusVisible&&r.setState({isFocusVisible:!1}),r.setState({focusedRadioIndex:-1}),r.props.onBlur&&r.props.onBlur(e)})),r}var r,o,n;return function(e,t){if("function"!==typeof t&&null!==t)throw new TypeError("Super expression must either be null or a function");e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:!0,configurable:!0}}),t&&A(e,t)}(t,e),r=t,(o=[{key:"componentDidMount",value:function(){}},{key:"render",value:function(){var e=this,t=this.props.overrides,r=void 0===t?{}:t,o=C(Object(p.c)(r.RadioGroupRoot,O),2),n=o[0],i=o[1];return s.createElement(n,S({id:this.props.id,role:"radiogroup","aria-describedby":this.props["aria-describedby"],"aria-errormessage":this.props["aria-errormessage"],"aria-invalid":this.props.error||this.props.isError||null,"aria-label":this.props["aria-label"],"aria-labelledby":this.props["aria-labelledby"],$align:this.props.align,$disabled:this.props.disabled,$isError:this.props.error||this.props.isError,$error:this.props.error||this.props.isError,$required:this.props.required},i),s.Children.map(this.props.children,(function(t,r){if(!s.isValidElement(t))return null;var o=e.props.value===t.props.value;return s.cloneElement(t,{align:e.props.align,autoFocus:e.props.autoFocus,checked:o,disabled:e.props.disabled||t.props.disabled,isError:e.props.isError,error:e.props.error,isFocused:e.state.focusedRadioIndex===r,isFocusVisible:e.state.isFocusVisible,tabIndex:0===r&&!e.props.value||o?"0":"-1",labelPlacement:e.props.labelPlacement,name:e.props.name,onBlur:function(t){return e.handleBlur(t,r)},onFocus:function(t){return e.handleFocus(t,r)},onChange:e.props.onChange,onMouseEnter:e.props.onMouseEnter,onMouseLeave:e.props.onMouseLeave,overrides:x({},e.props.overrides,{},t.props.overrides)})})))}}])&&L(r.prototype,o),n&&L(r,n),t}(s.Component);I(H,"defaultProps",{name:"",value:"",disabled:!1,autoFocus:!1,labelPlacement:"right",align:"vertical",isError:!1,error:!1,required:!1,onChange:function(){},onMouseEnter:function(){},onMouseLeave:function(){},onFocus:function(){},onBlur:function(){},overrides:{}});var _=H;function U(e){return(U="function"===typeof Symbol&&"symbol"===typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"===typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e})(e)}function z(){return(z=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var o in r)Object.prototype.hasOwnProperty.call(r,o)&&(e[o]=r[o])}return e}).apply(this,arguments)}function N(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){if(!(Symbol.iterator in Object(e))&&"[object Arguments]"!==Object.prototype.toString.call(e))return;var r=[],o=!0,n=!1,i=void 0;try{for(var a,s=e[Symbol.iterator]();!(o=(a=s.next()).done)&&(r.push(a.value),!t||r.length!==t);o=!0);}catch(c){n=!0,i=c}finally{try{o||null==s.return||s.return()}finally{if(n)throw i}}return r}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance")}()}function W(e,t){if(!(e instanceof t))throw new TypeError("Cannot call a class as a function")}function q(e,t){for(var r=0;r<t.length;r++){var o=t[r];o.enumerable=o.enumerable||!1,o.configurable=!0,"value"in o&&(o.writable=!0),Object.defineProperty(e,o.key,o)}}function G(e,t){return!t||"object"!==U(t)&&"function"!==typeof t?K(e):t}function J(e){return(J=Object.setPrototypeOf?Object.getPrototypeOf:function(e){return e.__proto__||Object.getPrototypeOf(e)})(e)}function K(e){if(void 0===e)throw new ReferenceError("this hasn't been initialised - super() hasn't been called");return e}function Q(e,t){return(Q=Object.setPrototypeOf||function(e,t){return e.__proto__=t,e})(e,t)}function X(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}var Y=function(e){function t(){var e,r;W(this,t);for(var o=arguments.length,n=new Array(o),i=0;i<o;i++)n[i]=arguments[i];return X(K(r=G(this,(e=J(t)).call.apply(e,[this].concat(n)))),"state",{isActive:!1,isHovered:!1}),X(K(r),"onMouseEnter",(function(e){r.setState({isHovered:!0}),r.props.onMouseEnter&&r.props.onMouseEnter(e)})),X(K(r),"onMouseLeave",(function(e){r.setState({isHovered:!1}),r.props.onMouseLeave&&r.props.onMouseLeave(e)})),X(K(r),"onMouseDown",(function(e){r.setState({isActive:!0}),r.props.onMouseDown&&r.props.onMouseDown(e)})),X(K(r),"onMouseUp",(function(e){r.setState({isActive:!1}),r.props.onMouseUp&&r.props.onMouseUp(e)})),r}var r,o,n;return function(e,t){if("function"!==typeof t&&null!==t)throw new TypeError("Super expression must either be null or a function");e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:!0,configurable:!0}}),t&&Q(e,t)}(t,e),r=t,(o=[{key:"componentDidMount",value:function(){this.props.autoFocus&&this.props.inputRef.current&&this.props.inputRef.current.focus()}},{key:"render",value:function(){var e,t=this.props.overrides,r=void 0===t?{}:t,o=N(Object(p.c)(r.Root,j),2),n=o[0],i=o[1],a=N(Object(p.c)(r.Label,F),2),c=a[0],l=a[1],u=N(Object(p.c)(r.Input,$),2),d=u[0],f=u[1],h=N(Object(p.c)(r.Description,R),2),b=h[0],m=h[1],v=N(Object(p.c)(r.RadioMarkInner,w),2),g=v[0],y=v[1],O=N(Object(p.c)(r.RadioMarkOuter,k),2),P=O[0],E=O[1],S={$align:this.props.align,$checked:this.props.checked,$disabled:this.props.disabled,$hasDescription:!!this.props.description,$isActive:this.state.isActive,$isError:this.props.isError,$error:this.props.error,$isFocused:this.props.isFocused,$isFocusVisible:this.props.isFocused&&this.props.isFocusVisible,$isHovered:this.state.isHovered,$labelPlacement:this.props.labelPlacement,$required:this.props.required,$value:this.props.value},M=s.createElement(c,z({},S,l),this.props.children);return s.createElement(s.Fragment,null,s.createElement(n,z({"data-baseweb":"radio",onMouseEnter:this.onMouseEnter,onMouseLeave:this.onMouseLeave,onMouseDown:this.onMouseDown,onMouseUp:this.onMouseUp},S,i),("top"===(e=this.props.labelPlacement)||"left"===e)&&M,s.createElement(P,z({},S,E),s.createElement(g,z({},S,y))),s.createElement(d,z({"aria-invalid":this.props.error||this.props.isError||null,checked:this.props.checked,disabled:this.props.disabled,name:this.props.name,onBlur:this.props.onBlur,onFocus:this.props.onFocus,onChange:this.props.onChange,ref:this.props.inputRef,required:this.props.required,tabIndex:this.props.tabIndex,type:"radio",value:this.props.value},S,f)),function(e){return"bottom"===e||"right"===e}(this.props.labelPlacement)&&M),!!this.props.description&&s.createElement(b,z({},S,m),this.props.description))}}])&&q(r.prototype,o),n&&q(r,n),t}(s.Component);X(Y,"defaultProps",{overrides:{},checked:!1,disabled:!1,autoFocus:!1,inputRef:s.createRef(),align:"vertical",isError:!1,error:!1,onChange:function(){},onMouseEnter:function(){},onMouseLeave:function(){},onMouseDown:function(){},onMouseUp:function(){},onFocus:function(){},onBlur:function(){}});var Z=Y,ee=r(137),te=r(138),re=r(69),oe=r(5),ne=function(e){Object(i.a)(r,e);var t=Object(a.a)(r);function r(){var e;Object(o.a)(this,r);for(var n=arguments.length,i=new Array(n),a=0;a<n;a++)i[a]=arguments[a];return(e=t.call.apply(t,[this].concat(i))).state={value:e.props.value},e.onChange=function(t){var r=parseInt(t.target.value,10);e.setState({value:r},(function(){return e.props.onChange(r)}))},e.render=function(){var t=e.props,r=t.disabled,o=t.theme,n=t.width,i=t.help,a=t.label,s=o.colors,c=o.radii,u={width:n},p=r,d=Object(l.a)(e.props.options);return 0===d.length&&(d.push("No options to select."),p=!0),Object(oe.jsxs)("div",{className:"row-widget stRadio",style:u,children:[Object(oe.jsx)(ee.d,{label:a,children:i&&Object(oe.jsx)(ee.c,{children:Object(oe.jsx)(te.a,{content:i,placement:re.b.TOP_RIGHT})})}),Object(oe.jsx)(_,{onChange:e.onChange,value:e.state.value.toString(),disabled:p,children:d.map((function(e,t){return Object(oe.jsx)(Z,{value:t.toString(),overrides:{Root:{style:function(e){return{marginBottom:0,marginTop:0,paddingLeft:0,paddingRight:"2px",backgroundColor:e.$isFocused?s.transparentDarkenedBgMix60:"",borderTopLeftRadius:c.md,borderTopRightRadius:c.md,borderBottomLeftRadius:c.md,borderBottomRightRadius:c.md}}},RadioMarkOuter:{style:function(e){return{backgroundColor:e.$checked&&!p?s.primary:s.fadedText40}}},RadioMarkInner:{style:function(e){var t=e.$checked;return{height:t?"6px":"16px",width:t?"6px":"16px"}}},Label:{style:{color:s.bodyText}}},children:e},t)}))})]})},e}return Object(n.a)(r,[{key:"componentDidUpdate",value:function(e){e.value!==this.props.value&&this.props.value!==this.state.value&&this.setState({value:this.props.value})}}]),r}(c.a.PureComponent),ie=Object(u.withTheme)(ne),ae=r(188),se=function(e){Object(i.a)(r,e);var t=Object(a.a)(r);function r(){var e;Object(o.a)(this,r);for(var n=arguments.length,i=new Array(n),a=0;a<n;a++)i[a]=arguments[a];return(e=t.call.apply(t,[this].concat(i))).formClearHelper=new ae.b,e.state={value:e.initialValue},e.commitWidgetValue=function(t){e.props.widgetMgr.setIntValue(e.props.element,e.state.value,t)},e.onFormCleared=function(){e.setState({value:e.props.element.default},(function(){return e.commitWidgetValue({fromUi:!0})}))},e.onChange=function(t){e.setState({value:t},(function(){return e.commitWidgetValue({fromUi:!0})}))},e.render=function(){var t=e.props,r=t.disabled,o=t.element,n=t.width,i=t.widgetMgr,a=o.options,s=o.label,c=o.help;return e.formClearHelper.manageFormClearListener(i,o.formId,e.onFormCleared),Object(oe.jsx)(ie,{label:s,onChange:e.onChange,options:a,width:n,disabled:r,value:e.state.value,help:c})},e}return Object(n.a)(r,[{key:"initialValue",get:function(){var e=this.props.widgetMgr.getIntValue(this.props.element);return void 0!==e?e:this.props.element.default}},{key:"componentDidMount",value:function(){this.props.element.setValue?this.updateFromProtobuf():this.commitWidgetValue({fromUi:!1})}},{key:"componentDidUpdate",value:function(){this.maybeUpdateFromProtobuf()}},{key:"componentWillUnmount",value:function(){this.formClearHelper.disconnect()}},{key:"maybeUpdateFromProtobuf",value:function(){this.props.element.setValue&&this.updateFromProtobuf()}},{key:"updateFromProtobuf",value:function(){var e=this,t=this.props.element.value;this.props.element.setValue=!1,this.setState({value:t},(function(){e.commitWidgetValue({fromUi:!1})}))}}]),r}(c.a.PureComponent)}}]);